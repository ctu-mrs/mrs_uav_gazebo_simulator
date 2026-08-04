#!/usr/bin/env python3
"""Open-loop FCU throttle ramp for the ArduPilot x500 backend.

Arms the SITL directly (bypasses MRS managers) and ramps commanded thrust,
printing per-motor SERVO_OUTPUT_RAW pwm, the CW/CCW split, and relative
altitude at each step. Used to diagnose the CW/CCW force/yaw imbalance that
prevents liftoff.

Usage: python3 throttle_ramp.py [max_thrust] [ramp_seconds]
"""
import sys, time
sys.path.insert(0, '/home/nekovfra/ardupilot/modules/mavlink')
from pymavlink import mavutil

MAX_THRUST = float(sys.argv[1]) if len(sys.argv) > 1 else 0.55
RAMP_SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0

conn = mavutil.mavlink_connection('udpin:0.0.0.0:14570')
conn.wait_heartbeat(timeout=15)
print('hb ok, mode:', conn.flightmode, flush=True)
# request SERVO_OUTPUT_RAW (36) streaming (SET_MESSAGE_INTERVAL) and fallback stream
conn.mav.command_long_send(conn.target_system, conn.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 36, 200000,
    0, 0, 0, 0, 0)
conn.mav.request_data_stream_send(2, 1, 0, 4, 1)
time.sleep(1)

def pump(seconds, thrust=None, report=False):
    t0 = time.time(); last_hb = 0; last_rep = 0
    while time.time() - t0 < seconds:
        if time.time() - last_hb > 0.1:
            conn.mav.heartbeat_send(6, 8, 0, 0, 0)
            if thrust is not None:
                conn.mav.set_attitude_target_send(
                    0, conn.target_system, conn.target_component, 0,
                    [1.0, 0.0, 0.0, 0.0], 0.0, 0.0, 0.0, thrust)
            last_hb = time.time()
        msg = conn.recv_match(blocking=False, timeout=0.005)
        if msg is None: continue
        if msg.get_type() == 'STATUSTEXT':
            print('TEXT:', msg.text, flush=True)
        if report and msg.get_type() == 'SERVO_OUTPUT_RAW' and time.time()-last_rep > 0.5:
            last_rep = time.time()
            c = [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw]
            print(f'  thrust={thrust:.2f}  servo={c[0]}/{c[1]}/{c[2]}/{c[3]} '
                  f'(ccw={c[0]}/{c[1]} cw={c[2]}/{c[3]})', flush=True)
    return

pump(2)
print('GUIDED...')
conn.mav.command_long_send(conn.target_system, conn.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 4, 0, 0, 0, 0, 0)
pump(1)
print('ARM...')
conn.mav.command_long_send(conn.target_system, conn.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
pump(3)
print('armed:', bool(conn.motors_armed()), flush=True)

print(f'ramp thrust 0 -> {MAX_THRUST} over {RAMP_SECS}s')
t0 = time.time()
while time.time() - t0 < RAMP_SECS:
    th = MAX_THRUST * (time.time() - t0) / RAMP_SECS
    pump(0.05, thrust=th, report=True)
print(f'hold thrust {MAX_THRUST} for 8s')
pump(8, thrust=MAX_THRUST, report=True)
print('cut thrust to 0')
pump(2, thrust=0.0)
print('--- final alt ---')
conn.mav.request_data_stream_send(conn.target_system, conn.target_component, 0, 5, 1)
t0 = time.time()
while time.time() - t0 < 2:
    msg = conn.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True, timeout=2)
    if msg:
        print('relative_alt:', msg.relative_alt / 1000.0, flush=True)
        break
print('DISARM')
conn.mav.command_long_send(conn.target_system, conn.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
