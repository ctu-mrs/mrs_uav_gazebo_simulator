#!/usr/bin/env python3
"""Watch the SITL MAVLink stream during a takeoff attempt.

Logs ATTITUDE (roll/pitch/yaw), SERVO_OUTPUT_RAW (mixer pwms) and STATUSTEXT
with timestamps to takeoff_watch.log in this directory. Run it in a session
pane just before triggering atomic_takeoff.sh.

Usage: python3 takeoff_watch.py [seconds]
"""
import sys, time, os

sys.path.insert(0, '/home/nekovfra/ardupilot/modules/mavlink')
from pymavlink import mavutil

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 45.0
LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'takeoff_watch.log')

conn = mavutil.mavlink_connection('udpin:0.0.0.0:14570')
conn.wait_heartbeat(timeout=30)

# ArduPilot ignores the legacy REQUEST_DATA_STREAM on this link; use
# MAV_CMD_SET_MESSAGE_INTERVAL (511) per message instead.
def set_interval(msg_id, hz):
    conn.mav.command_long_send(conn.target_system, conn.target_component, 511, 0, msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)

for _ in range(3):  # repeat in case a packet is dropped
    set_interval(30, 50)   # ATTITUDE
    set_interval(36, 50)   # SERVO_OUTPUT_RAW
    set_interval(33, 10)   # GLOBAL_POSITION_INT
    time.sleep(0.1)
time.sleep(0.5)

t0 = time.time()
n = 0
print(f'logging {DURATION} s to {LOG}')
with open(LOG, 'w') as f:
    f.write('# t roll_deg pitch_deg yaw_deg pwm1 pwm2 pwm3 pwm4\n')
    while time.time() - t0 < DURATION:
        msg = conn.recv_match(type=['ATTITUDE', 'SERVO_OUTPUT_RAW', 'STATUSTEXT'], blocking=True, timeout=0.2)
        if not msg:
            continue
        t = time.time() - t0
        mt = msg.get_type()
        if mt == 'ATTITUDE':
            if n % 5 == 0:
                print(f'{t:6.2f} r={msg.roll*57.2958:7.2f} p={msg.pitch*57.2958:7.2f} y={msg.yaw*57.2958:7.2f}')
            att = (msg.roll * 57.2958, msg.pitch * 57.2958, msg.yaw * 57.2958)
            n += 1
            if n % 10 == 0:
                f.write(f'{t:7.2f} {att[0]:8.2f} {att[1]:8.2f} {att[2]:8.2f}\n')
        elif mt == 'SERVO_OUTPUT_RAW':
            f.write(f'{t:7.2f} servo {msg.servo1_raw} {msg.servo2_raw} {msg.servo3_raw} {msg.servo4_raw}\n')
        elif mt == 'STATUSTEXT':
            line = f'{t:7.2f} TEXT: {msg.text.rstrip()}'
            print(line)
            f.write(line + '\n')
print('done')
