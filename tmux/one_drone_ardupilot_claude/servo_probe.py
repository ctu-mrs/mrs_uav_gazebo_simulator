import sys, time
sys.path.insert(0, '/home/nekovfra/ardupilot/modules/mavlink')
from pymavlink import mavutil

conn = mavutil.mavlink_connection('udpin:0.0.0.0:14570')
conn.wait_heartbeat(timeout=15)
conn.mav.request_data_stream_send(2, 1, 0, 4, 1)
t0 = time.time()
while time.time() - t0 < 8:
    msg = conn.recv_match(type=['SERVO_OUTPUT_RAW', 'STATUSTEXT'], blocking=True, timeout=2)
    if not msg:
        continue
    if msg.get_type() == 'SERVO_OUTPUT_RAW':
        print('SERVO', msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw)
    elif msg.get_type() == 'STATUSTEXT':
        print('TEXT:', msg.text)
