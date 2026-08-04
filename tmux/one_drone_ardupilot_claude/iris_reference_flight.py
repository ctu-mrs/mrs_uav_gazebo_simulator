import sys, time
sys.path.insert(0, '/home/nekovfra/ardupilot/modules/mavlink')
from pymavlink import mavutil

conn = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
conn.wait_heartbeat(timeout=20)
print('hb ok, mode:', conn.flightmode)
conn.mav.request_data_stream_send(1, 1, 0, 5, 1)
time.sleep(1)

def send_hb_and_drain(seconds):
    t0 = time.time()
    while time.time() - t0 < seconds:
        conn.mav.heartbeat_send(6, 8, 0, 0, 0)
        msg = conn.recv_match(blocking=True, timeout=0.5)
        if msg and msg.get_type() == 'STATUSTEXT':
            print('TEXT:', msg.text)

# EKF warmup
send_hb_and_drain(10)
print('GUIDED...')
conn.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 4, 0, 0, 0, 0, 0)
send_hb_and_drain(2)
print('mode now:', conn.flightmode)
print('ARM...')
conn.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
send_hb_and_drain(3)
print('armed?', conn.motors_armed())
print('TAKEOFF...')
conn.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 5)
t0 = time.time()
while time.time() - t0 < 25:
    conn.mav.heartbeat_send(6, 8, 0, 0, 0)
    msg = conn.recv_match(blocking=True, timeout=1)
    if msg:
        t = msg.get_type()
        if t == 'STATUSTEXT':
            print('TEXT:', msg.text)
        elif t == 'GLOBAL_POSITION_INT':
            print(f'alt={msg.relative_alt / 1000.0:.2f}m')
        elif t == 'SERVO_OUTPUT_RAW':
            print('servo', msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw)
        time.sleep(0.3)
