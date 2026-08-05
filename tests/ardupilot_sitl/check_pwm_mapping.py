#!/usr/bin/env python3
"""PWM/thrust-mapping verification for the ArduPilot SITL backend.

Drives a REAL ArduCopter SITL over its serial1 GCS UDP link (pymavlink,
GUIDED + SET_ATTITUDE_TARGET thrust) against the x500 ArduPilotPlugin model.
This isolates and measures the exact chain the user flagged:

    MRS thrust setpoint -> ArduPilot mixer pwm -> plugin omega -> thrust

and verifies the two suspects from the session:
  * pwm RANGE  : with MOT_SPIN_MIN=SPIN_ARM=THST_EXPO=0 the servo output must
                 be pwm = PWM_MIN + thrust*(PWM_MAX-PWM_MIN), i.e. a true zero
                 at thrust 0 (motors stop) and linear gain afterwards. The old
                 defaults (SPIN_MIN 0.15 / EXPO 0.65) push the floor up so a
                 0.2 thrust command already spins above the cutoff.
  * FORCE      : the vehicle must NOT lift off below the MRS hover throttle
                 (~0.44), must lift off just above it, proving total thrust
                 crosses the weight exactly there (CW/CCW forces balanced).

Light version: gz + directly-launched SITL, no MRS core / estimator overhead
(the MRS path is exercised separately by the full headless validation).

Usage:  ./check_pwm_mapping.py        (sources the workspace itself, cleans up)
Exit 0 = all checks passed.
"""

import json
import os
import subprocess
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RUN_DIR = os.path.join(SCRIPT_DIR, '.run')
PACKAGE_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..'))
ARDUPILOT_HOME = os.path.expanduser(os.environ.get('ARDUPILOT_HOME', '~/ardupilot'))

sys.path.insert(0, os.path.join(ARDUPILOT_HOME, 'modules', 'mavlink'))
sys.path.insert(0, SCRIPT_DIR)
from pymavlink import mavutil  # noqa: E402
from check_render import render_x500  # noqa: E402
import check_motor_physics as phys  # noqa: E402

URDG_BINARY = os.path.join(ARDUPILOT_HOME, 'build', 'sitl', 'bin', 'arducopter')
PARM_FILE = os.path.join(PACKAGE_DIR, 'config', 'ardupilot', 'mrs_copter_defaults.parm')
HOME = '47.397743,8.545594,340,0'
QGC_PORT = 14560              # serial1 udpclient port for instance 0
FDM_PORT = 9002               # fdm_port_in for instance 0

PWM_MIN, PWM_MAX = 1100, 1900
HOVER_THRUST_MRS = 0.44       # from MRS a/b curve
DRONE_MASS = phys.DRONE_MASS

failures = []
records = []


def check(cond, msg, val=None):
    tag = 'PASS' if cond else 'FAIL'
    print(f'  [{tag}] {msg}' + (f'  (got {val})' if val is not None else ''))
    if not cond:
        failures.append(msg)


def main():
    os.makedirs(RUN_DIR, exist_ok=True)
    env = phys.sourced_env()

    sdf_path = os.path.join(RUN_DIR, 'x500_ardupilot.sdf')
    with open(sdf_path, 'w') as f:
        f.write(render_x500({
            'name': phys.UAV_NAME, 'model': 'x500', 'use-ardupilot': {},
            'ardupilot_config': {'fdm_port_in': FDM_PORT},
        }))

    print('=== startup: gazebo + spawn + SITL ===')
    phys.cleanup_leftovers()
    time.sleep(2)

    gz_proc = sitl_proc = None
    try:
        gz_proc = phys.start_gz(env)
        phys.spawn_uav(env, sdf_path)
        print('  [info] spawned; launching ArduCopter SITL')
        sitl_cwd = os.path.join(RUN_DIR, 'sitl_uav0')
        os.makedirs(sitl_cwd, exist_ok=True)
        sitl_proc = subprocess.Popen(
            [URDG_BINARY, '--model', 'JSON', '--sim-address', '127.0.0.1', '-I', '0',
             '--defaults', PARM_FILE, '--sysid', '1', '--home', HOME,
             '--serial0=none', f'--serial1=udpclient:127.0.0.1:{QGC_PORT}'],
            cwd=sitl_cwd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        print('  [info] waiting for SITL heartbeat on the GCS link')
        conn = mavutil.mavlink_connection(f'udpin:0.0.0.0:{QGC_PORT}', autoreconnect=True)
        if not conn.wait_heartbeat(timeout=45):
            raise RuntimeError('no heartbeat from SITL')

        # ask for the message streams we need
        conn.mav.command_long_send(conn.target_system, conn.target_component,
                                   mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                   36, 200000, 0, 0, 0, 0, 0)   # SERVO_OUTPUT_RAW 5 Hz
        conn.mav.command_long_send(conn.target_system, conn.target_component,
                                   mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                   33, 200000, 0, 0, 0, 0, 0)   # GLOBAL_POSITION_INT 5 Hz
        time.sleep(1)

        def pump(seconds, thrust=None, collect=False):
            t0 = time.time()
            last_cmd = 0
            pwms, alts = [], []
            while time.time() - t0 < seconds:
                now = time.time()
                if now - last_cmd > 0.05:
                    conn.mav.heartbeat_send(6, 8, 0, 0, 0, 0)
                    if thrust is not None:
                        conn.mav.set_attitude_target_send(
                            0, conn.target_system, conn.target_component, 0,
                            [1.0, 0.0, 0.0, 0.0], 0.0, 0.0, 0.0, thrust)
                    last_cmd = now
                msg = conn.recv_match(blocking=False, timeout=0.02)
                if msg is None:
                    continue
                t = msg.get_type()
                if t == 'SERVO_OUTPUT_RAW':
                    pwms.append([msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw])
                elif t == 'GLOBAL_POSITION_INT':
                    alts.append(msg.relative_alt / 1000.0)
                elif t == 'STATUSTEXT':
                    print(f'    [fcu] {msg.text}')
            mean_pwm = [sum(c) / len(c) for c in zip(*pwms)] if pwms else [0, 0, 0, 0]
            return mean_pwm, (alts[-1] if alts else 0.0)

        # wait for EKF origin / GPS before arming (GUIDED needs it)
        print('  [info] waiting for position estimation (EKF origin)')
        pump(6)
        conn.mav.command_long_send(conn.target_system, conn.target_component,
                                   mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 4, 0, 0, 0, 0, 0)
        pump(1)
        print('  [info] mode:', conn.flightmode)

        steps = [0.0, 0.1, 0.2, 0.3, 0.40, 0.50, 0.60]
        print('  [info] arming and stepping thrust:', steps)
        # attempt to arm a few times; print prearm failures (STATUSTEXT) as we go
        conn.mav.command_long_send(conn.target_system, conn.target_component,
                                   mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                   1, 0, 0, 0, 0, 0, 0)
        pump(3)
        for _ in range(3):
            if conn.motors_armed():
                break
            conn.mav.command_long_send(conn.target_system, conn.target_component,
                                       mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                       1, 0, 0, 0, 0, 0, 0)
            pump(3)
        armed = bool(conn.motors_armed())
        check(armed, 'SITL armed in GUIDED')
        if not armed:
            raise RuntimeError('could not arm')

        liftoff_thrust = None
        for thrust in steps:
            mean_pwm, alt = pump(2.2, thrust=thrust, collect=True)
            om = phys.measure(1.0, env)
            row = {'thrust': thrust, 'pwm': [round(p, 1) for p in mean_pwm],
                   'alt': round(alt, 2), 'omega': {k: round(v, 1) for k, v in om['omega'].items()}}
            records.append(row)
            print(f"  thrust={thrust:.2f} pwm={[round(p) for p in mean_pwm]} "
                  f"alt={alt:.2f} om={[round(v) for v in om['omega'].values()]}")
            airborne = alt > 0.25
            if airborne and liftoff_thrust is None:
                liftoff_thrust = thrust

        # cut thrust, disarm
        pump(1.5, thrust=0.0)
        conn.mav.command_long_send(conn.target_system, conn.target_component,
                                   mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                   0, 0, 0, 0, 0, 0, 0)

        print('--- assertions ---')
        r0 = records[0]
        check(all(abs(p - PWM_MIN) <= 8 for p in r0['pwm']),
              f'thrust 0 -> pwm == {PWM_MIN} (motors stop, no cutoff-creep)', r0['pwm'])

        # linearity: pwm(t) ~ PWM_MIN + t*(PWM_MAX-PWM_MIN)
        for row in records:
            t = row['thrust']
            expected = PWM_MIN + t * (PWM_MAX - PWM_MIN)
            mean_all = sum(row['pwm']) / 4
            check(abs(mean_all - expected) <= 30,
                  f'thrust {t}: pwm linear 1100+800*t={expected:.0f} +/-30', round(mean_all, 1))

        # no CW/CCW pwm bias at each step
        max_spread = 0
        for row in records:
            spread = max(row['pwm']) - min(row['pwm'])
            max_spread = max(max_spread, spread)
        check(max_spread <= 40, 'per-step CW/CCW pwm balanced (spread <= 40)', round(max_spread, 1))

        # below-hover thrust must NOT lift off; ~hover must
        check(records[2]['alt'] < 0.25,
              'thrust 0.2 stays on the ground (below hover cutoff)', records[2]['alt'])
        check(liftoff_thrust is not None and 0.35 <= liftoff_thrust <= 0.60,
              f'liftoff near MRS hover throttle {HOVER_THRUST_MRS} (in [0.35,0.60])', liftoff_thrust)

    finally:
        for p in (sitl_proc, gz_proc):
            if p and p.poll() is None:
                p.terminate()
        phys.cleanup_leftovers()
        print('=== cleanup done ===')

    out = os.path.join(RUN_DIR, f'pwm_mapping_{time.strftime("%Y%m%d_%H%M%S")}.json')
    with open(out, 'w') as f:
        json.dump(records, f, indent=2)
    print(f'\nrecords -> {out}')
    if failures:
        print(f'\n{len(failures)} check(s) FAILED')
        for f_ in failures:
            print('  -', f_)
        return 1
    print('\nAll pwm mapping checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
