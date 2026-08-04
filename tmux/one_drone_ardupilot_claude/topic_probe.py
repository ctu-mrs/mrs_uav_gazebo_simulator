import rclpy
from rclpy.node import Node
import time

TOPICS = [
    ('/uav1/mavros/state', 'mavros_msgs/msg/State'),
    ('/uav1/mavros/imu/data_raw', 'sensor_msgs/msg/Imu'),
    ('/uav1/mavros/imu/data', 'sensor_msgs/msg/Imu'),
    ('/uav1/mavros/global_position/global', 'sensor_msgs/msg/NavSatFix'),
    ('/uav1/mavros/local_position/odom', 'nav_msgs/msg/Odometry'),
    ('/uav1/mavros/battery', 'sensor_msgs/msg/BatteryState'),
    ('/uav1/mavros/rc/in', 'mavros_msgs/msg/RCIn'),
]

def get_msg_class(cls_str):
    pkg, _, name = cls_str.replace('/', '.').partition('.msg.')
    mod = __import__(pkg + '.msg', fromlist=[name])
    return getattr(mod, name)

def main():
    rclpy.init()
    node = rclpy.create_node('probe_node')
    counts = {t: 0 for t, _ in TOPICS}
    subs = []
    for topic, cls_str in TOPICS:
        cls = get_msg_class(cls_str)
        def make_cb(t):
            def cb(msg):
                counts[t] += 1
            return cb
        subs.append(node.create_subscription(cls, topic, make_cb(topic), 10))
    t0 = time.time()
    while time.time() - t0 < 10:
        rclpy.spin_once(node, timeout_sec=0.5)
    for t, c in counts.items():
        print(f'{t}: {c}')
    node.destroy_node()
    rclpy.shutdown()

main()
