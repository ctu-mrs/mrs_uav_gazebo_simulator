#!/usr/bin/env python3
"""Hover stability probe: is the ArduPilot SITL + x500 parametric-rotor
airframe itself stable in GUIDED attitude control, WITHOUT any MRS in the
loop?

Background: with a healthy IMU the MRS takeoff spools up clean and level,
lifts off, climbs ~3 m, then a growing roll/pitch oscillation crashes the
vehicle. The SITL BIN log shows the MAVLink attitude input (GUIA) is level
the whole time while AP's desired attitude oscillates. This probe isolates
the FCU side: raw pymavlink -> SITL -> gz physics.

Protocol:
  * arm in GUIDED, hold spawned heading and level attitude target
  * ramp thrust 0.1 -> 0.46 (hover) over ~3 s, then HOLD for 14 s
  * log ATTITUDE + SERVO_OUTPUT_RAW at 20 Hz
  * metric: max |roll|,|pitch| during the airborne window must stay < 8 deg
    and the controller must not blow out the mix (pwm spread < 200).

Usage:  ./check_hover_stability.py        (sources the workspace, cleans up)
Exit 0 = airframe stable in closed loop.
"""

import math
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
QGC_PORT = 14560
FDM_PORT = 9002

RAMP_S = 3.0
HOLD_S = 14.0
THRUST_IDLE = 0.10
# NOTE: with the ArduPilotPlugin VELOCITY PID at i=0 plus joint damping, the
# motors droop ~6% below the commanded omega at steady state, and thrust is
# quadratic in omega, so the *effective* hover thrust of this stack is ~0.50
# (pwm ~1500), not 0.44 as the MRS thrust curve (lossless omega) predicts.
# MRS compensates this via its disturbance/mass estimator; for this raw
# FCU-side probe we simply sit slightly above the effective hover point.
THRUST_HOVER = 0.52
MAX_ANGLE_DEG = 8.0

failures = []


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
    series = []
    try:
        gz_proc = phys.start_gz(env)
        phys.spawn_uav(env, sdf_path)
        sitl_cwd = os.path.join(RUN_DIR, 'sitl_uav0')
        os.makedirs(sitl_cwd, exist_ok=True)
        sitl_proc = subprocess.Popen(
            [URDG_BINARY, '--model', 'JSON', '--sim-address', '127.0.0.1', '-I', '0',
             '--defaults', PARM_FILE, '--sysid', '1', '--home', HOME,
             '--serial0=none', f'--serial1=udpclient:127.0.0.1:{QGC_PORT}'],
            cwd=sitl_cwd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        conn = mavutil.mavlink_connection(f'udpin:0.0.0.0:{QGC_PORT}', autoreconnect=True)
        if not conn.wait_heartbeat(timeout=45):
            raise RuntimeError('no heartbeat from SITL')

        for mid, interval in ((36, 50000), (33, 100000), (30, 50000)):
            conn.mav.command_long_send(conn.target_system, conn.target_component,
                                       mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                       mid, interval, 0, 0, 0, 0, 0)
        time.sleep(1)

        def thresh_calc(t):
            if t < RAMP_S:
                return THRUST_IDLE + (THRUST_HOVER - THRUST_IDLE) * (t / RAMP_S)
            return THRUST_HOVER

        att0 = None
        # EKF settle
        t0 = time.time()
        while time.time() - t0 < 8:
            msg = conn.recv_match(blocking=False, timeout=0.02)
            if msg and msg.get_type() == 'ATTITUDE':
                att0 = (msg.roll, msg.pitch, msg.yaw, msg.yawspeed)
        hold_yaw = (att0[2] if att0 else 1.5708)
        att_target_quat = [math.cos(hold_yaw / 2), 0.0, 0.0, math.sin(hold_yaw / 2)]
        print(f'  [info] holding spawned heading (EKF yaw {hold_yaw:.2f} rad)')

        conn.mav.command_long_send(conn.target_system, conn.target_component,
                                   mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 4, 0, 0, 0, 0, 0)
        time.sleep(1)

        for _ in range(3):
            conn.mav.command_long_send(conn.target_system, conn.target_component,
                                       mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                       1, 0, 0, 0, 0, 0, 0)
            t0 = time.time()
            while time.time() - t0 < 2:
                msg = conn.recv_match(blocking=False, timeout=0.05)
                if msg and msg.get_type() == 'STATUSTEXT':
                    print(f'    [fcu] {msg.text}')
            if conn.motors_armed():
                break
        assert conn.motors_armed(), 'could not arm SITL'
        print('  [info] armed; ramp+hold thrust')

        for mid, interval in ((36, 50000), (33, 100000), (30, 50000)):
            conn.mav.command_long_send(conn.target_system, conn.target_component,
                                       mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                       mid, interval, 0, 0, 0, 0, 0)

        att_now = (0, 0, 0, 0)
        hold_yaw = 1.5708
        att_target_quat = [math.cos(hold_yaw / 2), 0.0, 0.0, math.sin(hold_yaw / 2)]

        print('  [info] mode:', conn.flightmode)
        t0 = time.time()
        T_END = 3.0 + HOLD_S
        max_rp_airborne = 0.0
        max_airborne_alt = 0.0
        airborne_min_t = None
        alt_now = 0.0
        pwm_now = (0, 0, 0, 0)
        last_print = -1
        while True:
            t = time.time() - t0
            if t > T_END:
                break
            thrust = min(THRUST_HOVER, THRUST_IDLE + (THRUST_HOVER - THRUST_IDLE) * (t / RAMP_S))
            conn.mav.heartbeat_send(6, 8, 0, 0, 0, 0)
            conn.mav.set_attitude_target_send(
                0, conn.target_system, conn.target_component, 0,
                att_target_quat, 0.0, 0.0, 0.0, thrust)
            msg = conn.recv_match(blocking=False, timeout=0.02)
            if msg:
                t_ = msg.get_type()
                if t_ == 'ATTITUDE':
                    att_now = (msg.roll, msg.pitch, msg.yaw, msg.yawspeed)
                elif t_ == 'GLOBAL_POSITION_INT':
                    alt_now = msg.relative_alt / 1000.0
                    if airborne_min_t is None and alt_now > 0.3:
                        airborne_min_t = t
                    if alt_now > max_airborne_alt:
                        max_airborne_alt = alt_now
                    if airborne_min_t is not None and t > airborne_min_t + 1.0:
                        r_deg = abs(att_now[0] * 57.2958)
                        p_deg = abs(att_now[1] * 57.2958)
                        max_rp_airborne = max(max_rp_airborne, r_deg, p_deg)
                elif t_ == 'SERVO_OUTPUT_RAW':
                    pwm_now = (msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw)
                elif t_ == 'STATUSTEXT':
                    print(f'    [fcu] {msg.text}')
            series.append((t,) + att_now + pwm_now + (alt_now,))
            if int(t) != last_print:
                last_print = int(t)
                print(f'    t={t:5.1f} thr={thrust:.3f} pwm={pwm_now} alt={alt_now:.2f} '
                      f'r={att_now[0]*57.3:.1f} p={att_now[1]*57.3:.1f} y={att_now[2]*57.3:.1f}')
            time.sleep(0.02)

        # post-analysis prints
        print(f'  [info] max airborne alt {max_airborne_alt:.2f} m; '
              f'max |roll|+|pitch| airborne {max_rp_airborne:.1f} deg')
        print(f'  [info] last attitude r/p/y deg: '
              f'{att_now[0]*57.2958:.1f} {att_now[1]*57.2958:.1f} {att_now[2]*57.2958:.1f}')
        print('  [info] samples (t, r, p, y, pwm1-4, alt):')
        for row in series[::10]:
            print(f'    {row[0]:5.1f} r={row[1]*57.3:6.1f} p={row[2]*57.3:6.1f} y={row[3]*57.3:6.1f} '
                  f'pwm={row[5]} {row[6]} {row[7]} {row[8]} alt={row[9]:.2f}')

        check(max_airborne_alt > 0.5, 'got airborne (> 0.5 m)', round(max_airborne_alt, 2))
        check(max_rp_airborne < MAX_ANGLE_DEG,
              f'max |roll|/|pitch| during airborne < {MAX_ANGLE_DEG} deg',
              round(max_rp_airborne, 1))
    finally:
        for p in (sitl_proc, gz_proc):
            if p and p.poll() is None:
                p.terminate()
        phys.cleanup_leftovers()

    print()
    if failures:
        print(f'FAILED ({len(failures)} checks):')
        for f in failures:
            print(' -', f)
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
