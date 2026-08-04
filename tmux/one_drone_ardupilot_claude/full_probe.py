import sys, time, math
sys.path.insert(0, '/home/nekovfra/ardupilot/modules/mavlink')
from pymavlink import mavutil

conn = mavutil.mavlink_connection('udpin:0.0.0.0:14570')
hb = conn.wait_heartbeat(timeout=20)
print('hb ok, mode:', conn.flightmode, 'target:', conn.target_system, conn.target_component)
conn.mav.request_data_stream_send(conn.target_system, conn.target_component, 0, 5, 1)
t0 = time.time()
n = 0
while time.time() - t0 < 12:
    conn.mav.heartbeat_send(6, 8, 0, 0, 0)
    msg = conn.recv_match(blocking=True, timeout=1)
    if msg and msg.get_type() == 'ATTITUDE':
        r, p, y = math.degrees(msg.roll), math.degrees(msg.pitch), math.degrees(msg.yaw)
        n += 1
        if n <= 3 or n % 8 == 0:
            print(f'roll={r:7.2f} pitch={p:7.2f} yaw={y:7.2f} rates={msg.rollspeed:+.2f},{msg.pitchspeed:+.2f},{msg.yawspeed:+.2f}')
