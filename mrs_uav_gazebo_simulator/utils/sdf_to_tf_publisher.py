import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation as R
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Transform

from mrs_uav_gazebo_simulator.utils.sdf_tf_enums import SensorLinkData, LinkToSensorData

class SingletonMeta(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[cls] = instance
        return cls._instances[cls]


class SdfTfPublisherSingleton(metaclass=SingletonMeta):
    def __init__(self, ros_node, base_frame, ignored_sensors):
        self._model_name = ""
        self._ignored_sensors = ignored_sensors
        self._base_frame = base_frame
        if self._base_frame is None:
            raise RuntimeError(f"[Sdf2Tf_Publisher] base_frame is not defined in the config file, cannot create tf publisher")
        self._ros_node = ros_node
        self._camera_types = ["camera", "rgbd_camera", "depth_camera"]

    def generate_tf_publishers(self, sdf_xml):
        root_xml = ET.fromstring(sdf_xml)
        model_xml = root_xml.find(".//model")
        self._model_name = model_xml.attrib["name"]

        links_to_sensors = self._detect_sensor_links(model_xml)
        sensors_tf = self._detect_sensors_transformations(links_to_sensors)
        self._generate_static_tf_broadcasters(sensors_tf)

    def _detect_sensors_transformations(self, links_to_sensors):
        T_World_Sensors = {}
        for link_name, data in links_to_sensors.items():
            pose_World_SensorLink_str = data[LinkToSensorData.LINK_POSE_STR]
            sensors = data[LinkToSensorData.SENSORS]

            for sensor in sensors:
                gz_frame_id = sensor[SensorLinkData.GZ_FRAME_ID]
                pose_Link_SensorOffset_str = sensor[SensorLinkData.SENSOR_OFFSET_POSE_STR]
                pose_SensorLink_OpticalFrame_str = sensor[SensorLinkData.GZ_FRAME_POSE_STR]

                # Compute transform of the sensor link with respect to the world frame
                if pose_World_SensorLink_str is None or (pose_World_SensorLink_str == ""):
                    self._ros_node.get_logger().info(
                        f"[Sdf2Tf_Publisher] Sensor link {link_name} has no pose, cannot create its tf publisher"
                        )
                    continue
                T_W_SensorLink = self._get_transform_from_string_pose(pose_World_SensorLink_str)

                # Compute transform of the sensor plugin with respect to the sensor link
                T_SensorLink_SensorPlugin = self._get_transform_from_string_pose(pose_Link_SensorOffset_str)

                # Compute transform of the optical frame with respect to the sensor link
                T_SensorPlugin_OpticalFrame = self._get_transform_from_string_pose(pose_SensorLink_OpticalFrame_str)
                
                T_W_SensorPlugin = T_W_SensorLink @ T_SensorLink_SensorPlugin
                T_W_OpticalFrame = T_W_SensorPlugin @ T_SensorPlugin_OpticalFrame
                T_World_Sensors[gz_frame_id] = T_W_OpticalFrame

                if self._has_optical_frame(pose_SensorLink_OpticalFrame_str):
                    T_World_Sensors[self._model_name + "/" + link_name] = T_W_SensorPlugin
            
        return T_World_Sensors

    def _get_transform_from_string_pose(self, pose_rpy_str: str) -> np.ndarray:
        T_frame = np.eye(4)
        if pose_rpy_str is not None and(pose_rpy_str != ""):
            pose_rpy = self._str_to_pose(pose_rpy_str)
            T_frame = self._pose_rpy_to_matrix(pose_rpy)
        return T_frame

    def _pose_rpy_to_matrix(self, pose_rpy):
        T_matrix = np.eye(4)
        T_matrix[:3, 3] = pose_rpy[:3]
        T_matrix[:3, :3] = R.from_euler("xyz", pose_rpy[3:], degrees=False).as_matrix()
        return T_matrix

    def _str_to_pose(self, pose_str):
        parts = pose_str.split()
        if len(parts) != 6:
            raise ValueError(f"[Sdf2Tf_Publisher] Expected 6 elements in pose string, got {len(parts)}: {pose_str}")
        x, y, z, roll, pitch, yaw = map(float, parts)
        return np.array([x, y, z, roll, pitch, yaw])
    
    def _has_optical_frame(self, pose_SensorLink_OpticalFrame_str):
        if pose_SensorLink_OpticalFrame_str is not None and (pose_SensorLink_OpticalFrame_str != ""):
            return True
        return False

    def _detect_sensor_links(self, model_xml):
        link_to_sensors = {}
        for link in model_xml.findall('.//link'):
            sensors = self._get_link_sensors(link)

            if len(sensors) > 0:
                link_sensor_name = link.get("name")
                link_sensor_pose_elem = link.find('pose')
                link_sensor_pose_str = link_sensor_pose_elem.text if link_sensor_pose_elem is not None else None

                sensors_within_link = []
                for sensor in sensors:
                    sensor_name = sensor.get("name")
                    sensor_offset_pose_str = sensor.findtext('pose')
                    gz_frame_name = sensor.findtext("gz_frame_id")

                    # Detect optical frames for cameras
                    pose_SensorLink_OpticalFrame_str = None
                    if sensor.get("type") in self._camera_types:
                        optical_frame = self._find_optical_frame_by_name(model_xml, gz_frame_name)
                        if optical_frame is None:
                            self._ros_node.get_logger().info(
                                f"[Sdf2Tf_Publisher] Link '{link_sensor_name}' may have an error in setting up the optical frame. Check the sdf file for the sensor."
                            )
                            gz_frame_name = ""
                        else:
                            pose_SensorLink_OpticalFrame_str = self._find_pose_by_link_name(model_xml, gz_frame_name)

                    sensors_within_link.append({
                        SensorLinkData.SENSOR_NAME : sensor_name,
                        SensorLinkData.SENSOR_TYPE : sensor.get("type"),
                        SensorLinkData.SENSOR_OFFSET_POSE_STR : sensor_offset_pose_str,
                        SensorLinkData.GZ_FRAME_POSE_STR : pose_SensorLink_OpticalFrame_str,
                        SensorLinkData.GZ_FRAME_ID : gz_frame_name,
                    })

                link_to_sensors[link_sensor_name] = {
                    LinkToSensorData.LINK_POSE_STR : link_sensor_pose_str,
                    LinkToSensorData.SENSORS : sensors_within_link,
                }
        return link_to_sensors

    def _get_link_sensors(self, link_xml):
        sensor_list = []
        for sensor_xml in link_xml.findall('.//sensor'):
            sensor_name = sensor_xml.get("name")
            if sensor_name not in self._ignored_sensors:
                sensor_list.append(sensor_xml)
        return sensor_list
    
    def _find_optical_frame_by_name(self, model_xml, optical_frame_name):
        for link in model_xml.findall('.//link'):
            if link.get("name") == optical_frame_name:
                return link
        return None

    def _find_pose_by_link_name(self, model_xml, link_name) -> str:
        if link_name is None:
            return ""
        for link in model_xml.findall('.//link'):
            if link.attrib["name"] == link_name:
                return link.findtext('pose')
        return ""
        
    def _generate_static_tf_broadcasters(self, sensors_tf):
        broadcaster = StaticTransformBroadcaster(self._ros_node)
        time_now = self._ros_node.get_clock().now().to_msg()

        transforms = []
        for frame_name, T_W_Sensor in sensors_tf.items():
            t = TransformStamped()
            t.header.stamp = time_now
            t.header.frame_id = self._model_name + "/" + self._base_frame
            t.child_frame_id = frame_name
            t.transform = self._matrix_to_tf_pose(T_W_Sensor)
            transforms.append(t)

        broadcaster.sendTransform(transforms)
        self._ros_node.get_logger().info(f"[Sdf2Tf_Publisher] Published {len(transforms)} static transforms relative to {self._base_frame}")

    def _matrix_to_tf_pose(self, T_W_Sensor: np.ndarray) -> Transform:
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
