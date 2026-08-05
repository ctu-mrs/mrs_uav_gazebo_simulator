#!/usr/bin/env python3
"""Physics verification of the x500 + ArduPilotPlugin motor/thrust pipeline.

Runs Gazebo headless with an x500 spawned in ArduPilot mode but WITHOUT the
SITL, so the fdm UDP port is free: this test itself injects synthetic servo
packets (the same servo_packet_16 wire format the ArduCopter SITL JSON
backend sends) and measures how the model responds on gz topics.

This isolates the physics chain from the flight-controller chain:

    pwm -> control-channel cmd -> joint velocity PID -> propeller omega
        -> per-blade LiftDrag force -> IMU proper acceleration

Assertions (theory-linked, see docs at the top of x500.sdf.jinja):
  1. idle: pwm == servo_min -> joints stay at ~0 rad/s (no Nyquist limit
     cycle), yaw rate ~0, IMU az ~ +9.81.
  2. symmetric mid-throttle: all joints track multiplier*(pwm-min)/(max-min)
     within tolerance, CCW joints positive, CW joints negative; all |omega|
     equal (no CW/CCW drive asymmetry); IMU az drop matches the LiftDrag
     lift integral within tolerance; yaw stays ~0 (blade-drag reaction
     torques cancel pairwise -> forward axes are mirrored correctly).
  3. single-rotor: each propeller, alone at the same pwm, produces the same
     lift (IMU az drop) within tolerance -> equal-force motors, catches a
     mis-mirrored CW/CCW blade configuration.
  4. saturated pwm -> net thrust exceeds weight: az gap goes strongly
     negative and the drone lifts off.

The test self-sources the workspace, starts/stops its own gz instance and
always cleans up gz/ROS leftovers afterwards (same process set as the tmux
session scripts).

Usage:
    ./check_motor_physics.py [--keep-running]

Exit code 0 = all checks passed.
"""

import argparse
import json
import os
import re
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..'))
WORKSPACE_DIR = os.path.abspath(os.path.join(PACKAGE_DIR, '..', '..'))
RUN_DIR = os.path.join(SCRIPT_DIR, '.run')

sys.path.insert(0, SCRIPT_DIR)
from check_render import render_x500  # noqa: E402

# ---- physics constants mirrored from x500.sdf.jinja -------------------------
SERVO_MIN, SERVO_MAX = 1100, 1900
MULTIPLIER = 780.0               # rad/s at full pwm (max_rot_velocity)
P_GAIN = 0.02                    # N*m per rad/s
JOINT_DAMPING = 0.002            # N*m s/rad
JOINT_FRICTION = 0.001           # N*m
PROP_RADIUS = 0.1751
CP = 2.0 / 3.0 * PROP_RADIUS     # center of pressure of one blade
BLADE_AREA = 0.00175
CL = 4.25 * 0.3                  # cla * a0, hover alpha regime
RHO = 1.2041
DRONE_MASS = 1.94 + 4 * 0.016076923076923075
GRAV = 9.81
FDM_PORT = 9002          # fdm_port_in of uav1 (9002 + 10*ID)
UAV_NAME = 'uav1'
WORLD = 'default'

# joint velocity equilibrium of the force PID under damping+friction load:
#   p_gain*(cmd - omega) = damping*omega + friction  (+ small blade drag)


def expected_omega(pwm):
    cmd = MULTIPLIER * (pwm - SERVO_MIN) / (SERVO_MAX - SERVO_MIN)
    return (P_GAIN * cmd - JOINT_FRICTION) / (P_GAIN + JOINT_DAMPING)


def expected_total_lift(omega):
    return 4 * 2 * CL * 0.5 * RHO * (omega * CP)**2 * BLADE_AREA


# -----------------------------------------------------------------------------

failures = []
measurements = {}


def check(condition, message, value=None):
    status = 'PASS' if condition else 'FAIL'
    suffix = f'  (measured {value})' if value is not None else ''
    print(f'  [{status}] {message}{suffix}')
    if not condition:
        failures.append(message)


def info(message):
    print(f'  [info] {message}')


# -----------------------------------------------------------------------------

def sourced_env():
    '''Environment with the workspace install space sourced (gz plugin paths).'''
    cmd = ('unset AMENT_TRACE_SETUP_FILES COLCON_TRACE; '
           f'source {WORKSPACE_DIR}/install/setup.bash >/dev/null 2>&1; env')
    out = subprocess.check_output(['bash', '-c', cmd], text=True)
    return dict(line.split('=', 1) for line in out.splitlines() if '=' in line.rstrip())


def cleanup_leftovers():
    '''Same kill set as the tmux session start scripts.'''
    subprocess.run(['tmux', '-L', 'mrs', 'kill-server'],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for pattern in ['gz sim', 'mavros_node', 'mrs_drone_spawner', 'build/sitl', 'component_container',
                    'rmw_zenohd']:
        subprocess.run(['pkill', '-9', '-f', pattern], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def start_gz(env, world=None):
    if world is None:
        world = subprocess.check_output(
            ['bash', '-c',
             f'source {WORKSPACE_DIR}/install/setup.bash >/dev/null 2>&1; '
             'echo $(ros2 pkg prefix mrs_gazebo_common_resources)/share/mrs_gazebo_common_resources'
             '/worlds/grass_plane.sdf'], text=True).strip()
    # the world inherits its sensor systems (imu-system, sensors-system, ...)
    # from this server config, exactly like mrs simulation.launch.py sets it up;
    # without it the IMU (and the whole ArduPilotPlugin JSON backend) stays silent
    server_config = subprocess.check_output(
        ['bash', '-c',
         f'source {WORKSPACE_DIR}/install/setup.bash >/dev/null 2>&1; '
         'echo $(ros2 pkg prefix mrs_uav_gazebo_simulator)/share/mrs_uav_gazebo_simulator'
         '/config/gazebo_server.config'], text=True).strip()
    env = dict(env)
    env['GZ_SIM_SERVER_CONFIG_PATH'] = server_config
    info(f'world: {world}')
    info(f'server config: {server_config}')
    proc = subprocess.Popen(['gz', 'sim', '-s', '-r', world], env=env,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    # wait until the world exists
    for _ in range(120):
        try:
            topics = subprocess.check_output(['gz', 'topic', '-l'], env=env, text=True,
                                             stderr=subprocess.DEVNULL)
            if f'/world/{WORLD}/pose/info' in topics:
                return proc
        except subprocess.CalledProcessError:
            pass
        time.sleep(0.5)
    raise RuntimeError('gazebo world did not come up')


def spawn_uav(env, sdf_path):
    req = (f'sdf_filename: "{sdf_path}"\n'
           f'pose {{ position {{ x: 0 y: 0 z: 0.3 }} orientation {{ w: 1 }} }}\n')
    out = subprocess.check_output(['gz', 'service', '-s', f'/world/{WORLD}/create',
                                   '--reqtype', 'gz.msgs.EntityFactory',
                                   '--reptype', 'gz.msgs.Boolean',
                                   '--req', req], env=env, text=True)
    if 'data: true' not in out:
        raise RuntimeError(f'spawn request failed: {out}')
    for _ in range(60):
        topics = subprocess.check_output(['gz', 'topic', '-l'], env=env, text=True,
                                         stderr=subprocess.DEVNULL)
        if f'/world/{WORLD}/model/{UAV_NAME}/joint_state' in topics:
            return
        time.sleep(0.5)
    raise RuntimeError('uav1 model topics never appeared')


# -----------------------------------------------------------------------------

class ServoDriver:
    '''Streams synthetic servo_packet_16 frames to the ArduPilotPlugin fdm port.'''

    MAGIC = 18458

    def __init__(self, port=FDM_PORT, rate_hz=50):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._addr = ('127.0.0.1', port)
        self._rate = rate_hz
        self._pwm = [SERVO_MIN] * 16
        self._frame = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def _loop(self):
        period = 1.0 / self._rate
        while not self._stop.is_set():
            # servo_packet_16 wire layout: u16 magic, u16 frame_rate,
            # u32 frame_count, u16 pwm[16] (little endian, 40 bytes)
            pkt = struct.pack('<HHI16H', self.MAGIC, self._rate, self._frame, *self._pwm)
            self._sock.sendto(pkt, self._addr)
            self._frame += 1
            self._stop.wait(period)

    def set_pwm(self, channel, pwm):
        self._pwm[channel] = pwm

    def set_all(self, pwm):
        for ch in range(4):
            self._pwm[ch] = pwm

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2)


def sample_gz(topic, seconds, env):
    '''Capture `seconds` worth of a gz topic, return the raw text of the tail.'''
    proc = subprocess.Popen(['gz', 'topic', '-e', '-t', topic], env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    try:
        out, _ = proc.communicate(timeout=seconds)
    except subprocess.TimeoutExpired:
        proc.send_signal(signal.SIGINT)
        proc.kill()
        out, _ = proc.communicate()
    return out


JOINT_STATE_TOPIC = f'/world/{WORLD}/model/{UAV_NAME}/joint_state'
IMU_TOPIC = f'/world/{WORLD}/model/{UAV_NAME}/link/base_link/sensor/imu_sensor/imu'


def last_vec3(text, block):
    '''Last (block {x y z}) triple in text; exact block name so the
    <block>_covariance fields do not shadow the real one.'''
    triples = re.findall(
        block + r'\s*\{\s*x: ([-0-9.eE+]+)\s+y: ([-0-9.eE+]+)\s+z: ([-0-9.eE+]+)', text)
    return [float(v) for v in triples[-1]] if triples else None


def measure(seconds, env):
    '''Sample joint_state and imu sequentially; return dict with omega/s, az, yaw_rate.'''
    raw_js = sample_gz(JOINT_STATE_TOPIC, seconds, env)
    raw_imu = sample_gz(IMU_TOPIC, seconds, env)

    omegas = {}
    for name, vel in re.findall(r'name: "(prop_\d_joint)"[\s\S]{0,600}?velocity: ([-0-9.eE+]+)', raw_js):
        omegas[name] = float(vel)   # last sample wins
    acc = last_vec3(raw_imu, 'linear_acceleration')
    gyro = last_vec3(raw_imu, 'angular_velocity')
    return {
        'omega': omegas,
        'az': acc[2] if acc else None,
        'yaw_rate': gyro[2] if gyro else None,
    }


# -----------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--keep-running', action='store_true',
                        help='do not kill gazebo at the end (debugging)')
    args = parser.parse_args()

    os.makedirs(RUN_DIR, exist_ok=True)
    env = sourced_env()

    sdf_path = os.path.join(RUN_DIR, 'x500_ardupilot.sdf')
    with open(sdf_path, 'w') as f:
        f.write(render_x500({
            'name': UAV_NAME, 'model': 'x500', 'use-ardupilot': {},
            'ardupilot_config': {'fdm_port_in': FDM_PORT},
        }))

    print('=== startup: cleanup + gazebo + spawn ===')
    cleanup_leftovers()
    time.sleep(2)

    gz_proc = None
    driver = ServoDriver()
    try:
        gz_proc = start_gz(env)
        spawn_uav(env, sdf_path)
        info('x500 (ardupilot) spawned; giving the ArduPilotPlugin 5 s to initialize')
        time.sleep(5)

        driver.start()

        # --- phase 1: idle --------------------------------------------------
        print('Phase 1: idle (pwm = servo_min on all channels)')
        driver.set_all(SERVO_MIN)
        time.sleep(3)
        m = measure(1.5, env)
        measurements['idle'] = m
        info(f"omegas: {m['omega']}")
        check(all(abs(v) < 5.0 for v in m['omega'].values()) and len(m['omega']) == 4,
              'idle: all propeller joints settle to ~0 rad/s (no PID limit cycle)', m['omega'])
        check(abs(m['yaw_rate']) < 0.05, 'idle: yaw rate ~0', m['yaw_rate'])
        check(9.4 < m['az'] < 10.2, 'idle: IMU az ~ +9.81 m/s^2', m['az'])

        # --- phase 2: symmetric mid-throttle --------------------------------
        pwm_mid = 1500
        print(f'Phase 2: symmetric mid-throttle (pwm = {pwm_mid} on all channels)')
        driver.set_all(pwm_mid)
        time.sleep(4)
        m = measure(2, env)
        measurements['symmetric'] = m
        w_expected = expected_omega(pwm_mid)
        signs = [1, 1, -1, -1]
        info(f'omegas: {m["omega"]}, expected magnitude ~{w_expected:.0f} rad/s')
        om = [m['omega'].get(f'prop_{k}_joint') for k in range(4)]
        check(all(o is not None for o in om), 'symmetric: all 4 joint velocities observed', om)
        for k, o in enumerate(om):
            check(signs[k] * o > 0, f'prop_{k}: spin direction correct '
                  f'({"CCW +z" if signs[k] > 0 else "CW -z"})', o)
            check(abs(abs(o) - w_expected) / w_expected < 0.10,
                  f'prop_{k}: |omega| tracks pwm ({w_expected:.0f} +/-10%)', o)
        spread = max(abs(o) for o in om) / min(abs(o) for o in om)
        check(spread < 1.08, 'symmetric: all |omega| equal +/-8% (no CW/CCW asymmetry)',
              f'ratio {spread:.3f}')
        az_drop = 9.81 - m['az']
        lift_meas = az_drop * DRONE_MASS
        lift_theory = expected_total_lift(w_expected)
        measurements['symmetric']['lift_measured'] = lift_meas
        measurements['symmetric']['lift_theory'] = lift_theory
        check(abs(lift_meas - lift_theory) / lift_theory < 0.25,
              'symmetric: measured lift matches LiftDrag theory +/-25% (force direction UP)',
              f'{lift_meas:.1f} N vs theory {lift_theory:.1f} N')
        check(m['az'] > 0.7, 'symmetric: vehicle produces less than hover thrust at pwm 1500 '
              '(stays on the ground)', m['az'])
        check(abs(m['yaw_rate']) < 0.08, 'symmetric: yaw rate ~0 (reaction torques cancel)',
              m['yaw_rate'])

        # --- phase 3: single-rotor equality ---------------------------------
        pwm_one = 1400
        print(f'Phase 3: per-rotor equality (one channel at {pwm_one}, rest at {SERVO_MIN})')
        drops = []
        for k in range(4):
            driver.set_all(SERVO_MIN)
            time.sleep(1.5)
            driver.set_pwm(k, pwm_one)
            time.sleep(2.5)
            m = measure(1.5, env)
            o = m['omega'].get(f'prop_{k}_joint', 0.0)
            drop = 9.81 - m['az']
            drops.append(drop)
            measurements[f'single_prop_{k}'] = {'omega': o, 'az_drop': drop}
            info(f'prop_{k}: omega {o:.0f} rad/s, az drop {drop:.2f} m/s^2')
            driver.set_pwm(k, SERVO_MIN)

        mean_drop = sum(drops) / 4
        check(all(d > 0.4 * mean_drop for d in drops),
              'single-rotor: every propeller produces UPWARD lift (az drop positive)', drops)
        check(all(abs(d - mean_drop) < 0.18 * mean_drop for d in drops),
              'single-rotor: all four propellers produce equal lift +/-18% '
              '(CW and CCW aerodynamics are truly mirroring)', drops)

        # --- phase 4: saturated pwm lifts the vehicle -----------------------
        pwm_max = 1700
        print(f'Phase 4: high throttle (pwm = {pwm_max}), expecting liftoff')
        driver.set_all(pwm_max)
        time.sleep(1.2)
        m = measure(0.8, env)
        measurements['saturated'] = m
        info(f"omegas: {m['omega']}, az: {m['az']}")
        check(m['az'] > 12.0, 'saturated: net upward acceleration (IMU reads az >> 9.81 '
              'while climbing, i.e. thrust exceeds weight)', m['az'])
        driver.set_all(SERVO_MIN)
        time.sleep(1)

    finally:
        driver.stop()
        if not args.keep_running:
            print('=== cleanup: killing gz and ROS leftovers ===')
            cleanup_leftovers()
        if gz_proc and gz_proc.poll() is None and not args.keep_running:
            gz_proc.terminate()

    # persist measurements
    out_path = os.path.join(RUN_DIR, f'motor_physics_{time.strftime("%Y%m%d_%H%M%S")}.json')
    with open(out_path, 'w') as f:
        json.dump(measurements, f, indent=2, default=str)
    print(f'\nmeasurements written to {out_path}')

    print()
    if failures:
        print(f'{len(failures)} check(s) FAILED:')
        for failure in failures:
            print(f'  - {failure}')
        return 1
    print('All motor physics checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
