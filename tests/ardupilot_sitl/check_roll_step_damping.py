#!/usr/bin/env python3
"""Roll-axis step-response probe for the ArduPilot SITL + x500 rotor stack.

The MRS-driven takeoff now lifts off and hovers ~4-5 s, but a ~2-4 Hz
roll/pitch oscillation grows until it crashes, while the FCU-side constant
command flight is rock solid. A plausible cause is an under-damped AP
ATTITUDE TRACKING loop on this plant (masked by never having to track a
command). This probe makes the AP attitude loop WORK: hold a hover, then
apply +-5 deg roll HYSTERESIS steps and see whether tracking is damped or
oscillatory.

Protocol (identical boot chain to check_hover_stability.py):
  * arm in GUIDED, hover thrust (level, hold yaw) for 3 s
  * roll reference: 0 -> +5 deg (1.5 s) -> -5 deg (1.5 s) -> 0 (1.5 s)
  * measure roll vs desired: overshoot and settling metric.

Passes if tracking stays within +-20% overshoot and settles (<1.5 deg
residual after 0.8 s) and no divergence/disarm.

Usage:  ./check_roll_step_damping.py   (sources the workspace, cleans up)
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

THRUST_HOVER = 0.50
ROLL_STEP_DEG = 5.0
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

        # optional param overrides from argv: ./check_roll_step_damping.py KEY=VAL,KEY=VAL
        for arg in sys.argv[1:]:
            if '=' in arg:
                k, v = arg.split('=', 1)
                conn.mav.param_set_send(conn.target_system, conn.target_component,
                                        k.encode(), float(v), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                time.sleep(0.3)
                conn.mav.param_request_read_send(conn.target_system, conn.target_component,
                                                 k.encode(), -1)
                t0 = time.time()
                readback = None
                while time.time() - t0 < 3:
                    msg = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
                    if msg and msg.param_id.rstrip('\x00') == k:
                        readback = msg.param_value
                        break
                print(f'  [info] param {k} = {v} (readback: {readback})')

        for mid, interval in ((36, 50000), (33, 100000), (30, 50000)):
            conn.mav.command_long_send(conn.target_system, conn.target_component,
                                       mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                       mid, interval, 0, 0, 0, 0, 0)
        time.sleep(1)

        att0 = (0, 0, 0, 0)
        t0 = time.time()
        while time.time() - t0 < 8:
            msg = conn.recv_match(blocking=False, timeout=0.02)
            if msg and msg.get_type() == 'ATTITUDE':
                att0 = (msg.roll, msg.pitch, msg.yaw, msg.yawspeed)
        hold_yaw = att0[2] if att0 else 1.5708

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
        print('  [info] armed; hover 3 s, then +-5 deg roll steps')

        hold_yaw = 1.5708
        phase_steps = [(0.0, 6.0), (ROLL_STEP_DEG, 1.5), (-ROLL_STEP_DEG, 1.5), (0.0, 2.0)]
        t_start = time.time()
        t = 0.0
        alt_now = 0.0
        att_now = (0, 0, hold_yaw, 0)
        pwm_now = (0, 0, 0, 0)
        steps = []
        for target, dur in phase_steps:
            t_end = time.time() + dur
            while time.time() < t_end:
                t = time.time() - t_start
                # quaternion for (roll=target, yaw=hold_yaw)
                roll = math.radians(target)
                cy, sy = math.cos(hold_yaw / 2), math.sin(hold_yaw / 2)
                cr, sr = math.cos(roll / 2), math.sin(roll / 2)
                # q = qyaw * qroll (roll about body x after yaw):
                # y component is +sy*sr for a true (roll, 0, yaw) euler target
                q = [cy * cr, cy * sr, sy * sr, sy * cr]  # w, x, y, z
                conn.mav.heartbeat_send(6, 8, 0, 0, 0, 0)
                # TYPE_MASK: env AP_TYPE_MASK (default 0 = use rates; 7 =
                # ignore all body rates, as the mrs hw api sends them).
                # THRUST modulated when AP_THRUST_MOD=a,f is set (MRS-like
                # +-0.06 at ~2-3 Hz zigzag during hover).
                thrust_val = THRUST_HOVER
                mod = os.environ.get('AP_THRUST_MOD', '')
                if mod:
                    amp, freq = (float(x) for x in mod.split(','))
                    thrust_val += amp * math.sin(2 * math.pi * freq * (t - 0.0))
                conn.mav.set_attitude_target_send(
                    0, conn.target_system, conn.target_component,
                    int(os.environ.get('AP_TYPE_MASK', '0')),
                    q, 0.0, 0.0, 0.0, thrust_val)
                msg = conn.recv_match(blocking=False, timeout=0.02)
                if msg:
                    tt = msg.get_type()
                    if tt == 'ATTITUDE':
                        att_now = (msg.roll, msg.pitch, msg.yaw, msg.yawspeed)
                    elif tt == 'GLOBAL_POSITION_INT':
                        alt_now = msg.relative_alt / 1000.0
                    elif tt == 'SERVO_OUTPUT_RAW':
                        pwm_now = (msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw)
                series.append((t, target, att_now[0] * 57.2958, att_now[1] * 57.2958, att_now[2] * 57.2958, alt_now,
                               pwm_now[0], pwm_now[2]))
                time.sleep(0.02)
        steps.append(target)

        # ---- analysis ----
        seg = lambda a, b, trg: [s for s in series if a <= s[0] < b]
        # overshoot in each step window
        max_over = 0.0
        for tseg in ((6.0, 7.5, ROLL_STEP_DEG), (7.5, 9.0, -ROLL_STEP_DEG)):
            a, b, trg = tseg
            sub = [(s[0], s[2]) for s in seg(a, b, None)]
            if sub:
                over = max(abs(r[1] - trg) for r in sub)
                max_over = max(max_over, over)
        residual = [abs(s[2]) for s in series if s[0] > 9.8]
        res_max = max(residual) if residual else 99
        max_alt = max(s[5] for s in series)
        min_alt = min(s[5] for s in series[100:])
        crash = max([abs(s[2]) for s in series] + [0]) > 45
        print(f'  [info] max overshoot beyond ref: {max_over:.1f} deg; '
              f'residual |roll| after t=9.8: {res_max:.1f} deg; alt range [{min_alt:.1f},{max_alt:.1f}]')
        # dump coarse series
        print('  [info] roll series (t, ref, roll, yaw, alt):')
        for s in series[::12]:
            print(f'    {s[0]:5.1f} ref={s[1]:6.1f} r={s[2]:7.1f} y={s[4]:6.1f} alt={s[5]:6.2f} pwm={s[6]}/{s[7]}')

        check(max_alt > 1.5, 'hover reached (~2 m)', round(max_alt, 2))
        check(not crash, 'no 45 deg tumble during steps')
        check(max_over < 2.0, f'tracking overshoot beyond +-5 deg ref < 2 deg', round(max_over, 1))
        check(res_max < 3.0, 'residual roll after settle < 3 deg', round(res_max, 1))
    finally:
        for p in (sitl_proc, gz_proc):
            if p and p.poll() is None:
                p.terminate()
        phys.cleanup_leftovers()

    if failures:
        print(f'\n{len(failures)} check(s) FAILED')
        for f_ in failures:
            print(' -', f_)
        return 1
    print('\nALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
