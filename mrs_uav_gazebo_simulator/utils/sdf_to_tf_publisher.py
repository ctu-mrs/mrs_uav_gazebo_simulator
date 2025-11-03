import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation as R
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Transform

class SingletonMeta(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[cls] = instance
        return cls._instances[cls]


class SdfTfPublisher(metaclass=SingletonMeta):
    def __init__(self, ros_node, base_link, ignored_sensors):
        self._model_name = ""
        self._ignored_sensors = ignored_sensors
        self._base_link = base_link
        if self._base_link is None:
            raise RuntimeError(f"[Sdf2Tf_Publisher] base_link (parent link) is not defined in the config file, cannot create tf publisher")
        self._ros_node = ros_node

    def generate_tf_publishers(self, sdf_xml):
        root_xml = ET.fromstring(sdf_xml)
        model_xml = root_xml.find(".//model")
        self._model_name = model_xml.attrib["name"]

        sensor_links_to_poses = self._detect_sensors(model_xml)
        sensors_tf = self._detect_sensors_transformations(sensor_links_to_poses)
        self._generate_static_tf_broadcasters(sensors_tf)

    def _detect_sensors_transformations(self, sensor_joints):
        sensors_Tf = {}
        for link_name, poses in sensor_joints.items():
            pose_str = poses["link_pose"]
            sensor_pose_str = poses["sensor_pose"]

            # Detect joint pose
            if pose_str is None or (pose_str == ""):
                self._ros_node.get_logger().info(f"[Sdf2Tf_Publisher] Link {link_name} has no pose, cannot create its tf publisher")
                continue
            link_pose_rpy = self._str_to_pose(pose_str)

            # Detect sensor offset (optional)
            if sensor_pose_str is None or (sensor_pose_str == ""):
                self._ros_node.get_logger().info(f"[Sdf2Tf_Publisher] Link {link_name} has no pose specified in its sensor plugin")
                sensor_pose_offset = np.zeros(6)
            else:
                sensor_pose_offset = self._str_to_pose(sensor_pose_str)

            sensors_Tf[link_name] = self._add_pose_with_offset(link_pose_rpy, sensor_pose_offset)

        return sensors_Tf

    def _add_pose_with_offset(self, link_pose, sensor_offset):
        T_W_Link = np.eye(4)
        T_W_Link[:3, 3] = link_pose[:3]
        T_W_Link[:3, :3] = R.from_euler("xyz", link_pose[3:]).as_matrix()

        T_Link_Sensor = np.eye(4)
        T_Link_Sensor[:3, 3] = sensor_offset[:3]
        T_Link_Sensor[:3, :3] = R.from_euler("xyz", sensor_offset[3:]).as_matrix()

        T_W_Sensor = T_W_Link@T_Link_Sensor
        return T_W_Sensor

    def _str_to_pose(self, pose_str):
        parts = pose_str.split()
        if len(parts) != 6:
            raise ValueError(f"[Sdf2Tf_Publisher] Expected 6 elements in pose string, got {len(parts)}: {pose_str}")

        x, y, z, roll, pitch, yaw = map(float, parts)
        return np.array([x, y, z, roll, pitch, yaw])

    def _detect_sensors(self, model_xml):
        sensor_links_to_poses = {}
        for link in model_xml.findall('.//link'):
            for sensor in link.findall('.//sensor'):
                if sensor.attrib["name"] not in self._ignored_sensors:
                    sensor_links_to_poses[link.attrib["name"]] = {
                        "link_pose" : link.findtext('pose'),
                        "sensor_pose" : sensor.findtext('pose')
                    }
        return sensor_links_to_poses

    def _generate_static_tf_broadcasters(self, sensors_tf):
        broadcaster = StaticTransformBroadcaster(self._ros_node)
        time_now = self._ros_node.get_clock().now().to_msg()

        transforms = []
        for link_name, T_W_Sensor in sensors_tf.items():
            t = TransformStamped()
            t.header.stamp = time_now
            t.header.frame_id = self._model_name + "/" + self._base_link
            t.child_frame_id = self._model_name + "/" + link_name

            t.transform = self._get_sensor_pose(T_W_Sensor)
            transforms.append(t)

        broadcaster.sendTransform(transforms)
        self._ros_node.get_logger().info(f"[Sdf2Tf_Publisher] Published {len(transforms)} static transforms relative to {self._base_link}")

    def _get_sensor_pose(self, T_W_Sensor: np.ndarray) -> Transform:
        pose = Transform()
        pose.translation.x = T_W_Sensor[0, 3]
        pose.translation.y = T_W_Sensor[1, 3]
        pose.translation.z = T_W_Sensor[2, 3]

        quat = R.from_matrix(T_W_Sensor[:3, :3]).as_quat()
        pose.rotation.x = quat[0]
        pose.rotation.y = quat[1]
        pose.rotation.z = quat[2]
        pose.rotation.w = quat[3]

        return pose
