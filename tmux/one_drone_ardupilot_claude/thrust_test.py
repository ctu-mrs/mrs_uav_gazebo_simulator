import sys, time, math
sys.path.insert(0, '/home/nekovfra/ardupilot/modules/mavlink')
from pymavlink import mavutil

conn = mavutil.mavlink_connection('udpin:0.0.0.0:14570')
conn.wait_heartbeat(timeout=15)
print('hb ok, mode:', conn.flightmode)

def pump(seconds, thrust=None):
    t0 = time.time()
    last_hb = 0
    while time.time() - t0 < seconds:
        if time.time() - last_hb > 0.1:
            conn.mav.heartbeat_send(6, 8, 0, 0, 0)
            if thrust is not None:
                # SET_ATTITUDE_TARGET: quaternion(yaw=0), thrust
                conn.mav.set_attitude_target_send(
                    0, conn.target_system, conn.target_component,
                    0,                    # type_mask
                    [1.0, 0.0, 0.0, 0.0], # quaternion wxyz = identity
                    0.0, 0.0, 0.0,        # rates
                    thrust)
            last_hb = time.time()
        msg = conn.recv_match(blocking=False, timeout=0.01)
        if msg and msg.get_type() == 'STATUSTEXT':
            print('TEXT:', msg.text)

pump(2)
print('GUIDED...'); conn.mav.command_long_send(conn.target_system, conn.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 4, 0, 0, 0, 0, 0); pump(1)
print('mode:', conn.flightmode)
print('ARM...'); conn.mav.command_long_send(conn.target_system, conn.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0); pump(3)
print('armed:', bool(conn.motors_armed()))

print('ramp thrust 0 -> 0.5 over 4s')
t0 = time.time()
while time.time() - t0 < 4:
    th = 0.5 * (time.time() - t0) / 4.0
    pump(0.05, thrust=th)
print('hold thrust 0.5 for 4s')
pump(4, thrust=0.5)
print('cut thrust to 0 for 2s')
pump(2, thrust=0.0)
print('done - check alt')
conn.mav.request_data_stream_send(conn.target_system, conn.target_component, 0, 5, 1)
t0 = time.time()
while time.time() - t0 < 2:
    msg = conn.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True, timeout=2)
    if msg:
        print('alt:', msg.relative_alt / 1000.0)
