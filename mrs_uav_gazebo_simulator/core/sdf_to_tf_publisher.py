import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation as R
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Transform

from mrs_uav_gazebo_simulator.utils.sdf_tf_enums import SensorLinkData, LinkToSensorData, TfData
from mrs_uav_gazebo_simulator.utils.spawner_types import GazeboSensors


class SingletonMeta(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[cls] = instance
        return cls._instances[cls]


class SdfTfPublisherSingleton(metaclass=SingletonMeta):

    # #{ __init__(self, ros_node, base_frame, ignored_sensors)
    def __init__(self, ros_node, base_frame, ignored_sensors):
        self._model_name = ""
        self._ignored_sensors = ignored_sensors
        self._base_frame = base_frame
        if self._base_frame is None:
            raise RuntimeError(
                f"[Sdf2Tf_Publisher] base_frame is not defined in the config file, cannot create tf publisher.")
        self._ros_node = ros_node
        self._camera_types = [GazeboSensors.CAMERA, GazeboSensors.RGBD_CAMERA, GazeboSensors.DEPTH_CAMERA]

        self._transformations = []
        self._broadcaster = StaticTransformBroadcaster(self._ros_node)

    # #}

    # #{ publish_sensor_tfs(self)
    def publish_sensor_tfs(self):
        if len(self._transformations) == 0:
            self._ros_node.get_logger().info(f"[Sdf2Tf_Publisher] There are no TFs to publish.")
            return
        self._generate_static_tf_broadcasters(self._transformations)

    # #}

    # #{ generate_sensor_tfs(self, sdf_xml)
    def generate_sensor_tfs(self, sdf_xml):
        root_xml = ET.fromstring(sdf_xml)
        model_xml = root_xml.find(".//model")
        self._model_name = model_xml.get("name")

        links_to_sensors = self._detect_sensor_links(model_xml)
        self._detect_sensors_transformations(links_to_sensors)

    # #}

    # #{ _detect_sensors_transformations(self, links_to_sensors)
    def _detect_sensors_transformations(self, links_to_sensors):
        for link_name, data in links_to_sensors.items():
            if not self._register_sensor_link_transform(link_name=link_name, data=data):
                self._ros_node.get_logger().info(
                    f"[Sdf2Tf_Publisher] Sensor link {link_name} has no pose, cannot create its tf publisher.")
                continue
            sensors = data[LinkToSensorData.SENSORS]
            for sensor in sensors:
                self._register_sensor_offset_transform(sensor_data=sensor, parent_frame=link_name)

                if self._has_optical_frame(sensor[SensorLinkData.OPTICAL_FRAME_POSE_STR]):
                    self._register_optical_frame_transform(sensor)

    # #}

    # #{ _register_sensor_link_transform(self, link_name, data)
    def _register_sensor_link_transform(self, link_name, data):
        # Publish transform of the sensor link with respect to the world frame
        pose_World_SensorLink_str = data[LinkToSensorData.LINK_POSE_STR]
        if pose_World_SensorLink_str is None or (pose_World_SensorLink_str == ""):
            return False
        T_W_SensorLink = self._get_transform_from_string_pose(pose_World_SensorLink_str)

        self._transformations.append({
            TfData.CHILD_FRAME: self._append_namespace(link_name),
            TfData.PARENT_FRAME: self._append_namespace(self._base_frame),
            TfData.TF_MATRIX: T_W_SensorLink
        })
        return True

    # #}

    # #{ _register_sensor_offset_transform(self, sensor_data, parent_frame)
    def _register_sensor_offset_transform(self, sensor_data, parent_frame):
        # Publish transform of the sensor plugin with respect to the sensor link
        T_SensorLink_SensorPlugin = self._get_transform_from_string_pose(
            sensor_data[SensorLinkData.SENSOR_OFFSET_POSE_STR])

        self._transformations.append({
            TfData.CHILD_FRAME: self._append_namespace(sensor_data[SensorLinkData.SENSOR_NAME]),
            TfData.PARENT_FRAME: self._append_namespace(parent_frame),
            TfData.TF_MATRIX: T_SensorLink_SensorPlugin
        })

    # #}

    # #{ _register_optical_frame_transform(self, sensor_data)
    def _register_optical_frame_transform(self, sensor_data):
        # Publish transform of the optical frame with respect to the sensor plugin
        T_SensorPlugin_OpticalFrame = self._get_transform_from_string_pose(
            sensor_data[SensorLinkData.OPTICAL_FRAME_POSE_STR])

        self._transformations.append({
            TfData.CHILD_FRAME:
            self._append_namespace(sensor_data[SensorLinkData.OPTICAL_FRAME_NAME]),
            TfData.PARENT_FRAME:
            self._append_namespace(sensor_data[SensorLinkData.SENSOR_NAME]),
            TfData.TF_MATRIX:
            T_SensorPlugin_OpticalFrame
        })

    # #}

    # #{ _append_namespace(self, frame: str) -> str
    def _append_namespace(self, frame: str) -> str:
        prefix = self._model_name + "/"
        if not frame.startswith(prefix):
            frame = prefix + frame
        return frame

    # #}

    # #{ _get_transform_from_string_pose(self, pose_rpy_str: str) -> np.ndarray
    def _get_transform_from_string_pose(self, pose_rpy_str: str) -> np.ndarray:
        T_frame = np.eye(4)
        if pose_rpy_str is not None and (pose_rpy_str != ""):
            pose_rpy = self._str_to_pose(pose_rpy_str)
            T_frame = self._pose_rpy_to_matrix(pose_rpy)
        return T_frame

    # #}

    # #{ _pose_rpy_to_matrix(self, pose_rpy: str) -> np.ndarray
    def _pose_rpy_to_matrix(self, pose_rpy: str) -> np.ndarray:
        T_matrix = np.eye(4)
        T_matrix[:3, 3] = pose_rpy[:3]
        T_matrix[:3, :3] = R.from_euler("xyz", pose_rpy[3:], degrees=False).as_matrix()
        return T_matrix

    # #}

    # #{ _str_to_pose(self, pose_str: str) -> np.ndarray
    def _str_to_pose(self, pose_str: str) -> np.ndarray:
        parts = pose_str.split()
        if len(parts) != 6:
            raise ValueError(f"[Sdf2Tf_Publisher] Expected 6 elements in pose string, got {len(parts)}: {pose_str}.")
        x, y, z, roll, pitch, yaw = map(float, parts)
        return np.array([x, y, z, roll, pitch, yaw])

    # #}

    # #{ _has_optical_frame(self, pose_SensorLink_OpticalFrame_str) -> bool
    def _has_optical_frame(self, pose_SensorLink_OpticalFrame_str) -> bool:
        if pose_SensorLink_OpticalFrame_str is not None and (pose_SensorLink_OpticalFrame_str != ""):
            return True
        return False

    # #}

    # #{ _detect_sensor_links(self, model_xml) -> dict
    def _detect_sensor_links(self, model_xml) -> dict:
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
                        else:
                            pose_SensorLink_OpticalFrame_str = self._find_pose_by_link_name(model_xml, gz_frame_name)

                    sensors_within_link.append({
                        SensorLinkData.SENSOR_NAME: sensor_name,
                        SensorLinkData.SENSOR_TYPE: sensor.get("type"),
                        SensorLinkData.SENSOR_OFFSET_POSE_STR: sensor_offset_pose_str,
                        SensorLinkData.OPTICAL_FRAME_POSE_STR: pose_SensorLink_OpticalFrame_str,
                        SensorLinkData.OPTICAL_FRAME_NAME: gz_frame_name,
                    })

                link_to_sensors[link_sensor_name] = {
                    LinkToSensorData.LINK_POSE_STR: link_sensor_pose_str,
                    LinkToSensorData.SENSORS: sensors_within_link,
                }
        return link_to_sensors

    # #}

    # #{ _get_link_sensors(self, link_xml) -> list
    def _get_link_sensors(self, link_xml) -> list:
        sensor_list = []
        for sensor_xml in link_xml.findall('.//sensor'):
            sensor_name = sensor_xml.get("name")
            if sensor_name not in self._ignored_sensors:
                sensor_list.append(sensor_xml)
        return sensor_list

    # #}

    # #{ _find_optical_frame_by_name(self, model_xml, optical_frame_name)
    def _find_optical_frame_by_name(self, model_xml, optical_frame_name):
        for link in model_xml.findall('.//link'):
            if link.get("name") == optical_frame_name:
                return link
        return None

    # #}

    # #{ _find_pose_by_link_name(self, model_xml, link_name) -> str
    def _find_pose_by_link_name(self, model_xml, link_name) -> str:
        if link_name is None:
            return ""
        for link in model_xml.findall('.//link'):
            if link.get("name") == link_name:
                return link.findtext('pose')
        return ""

    # #}

    # #{ _generate_static_tf_broadcasters(self, transformations)
    def _generate_static_tf_broadcasters(self, transformations):
        time_now = self._ros_node.get_clock().now().to_msg()

        tf_transforms = []
        for data in transformations:
            child_frame = data[TfData.CHILD_FRAME]
            parent_frame = data[TfData.PARENT_FRAME]
            T_matrix = data[TfData.TF_MATRIX]

            self._ros_node.get_logger().info(
                f"[Sdf2Tf_Publisher] Creating tf link from: {parent_frame} to: {child_frame}")

            t = TransformStamped()
            t.header.stamp = time_now
            t.header.frame_id = parent_frame
            t.child_frame_id = child_frame
            t.transform = self._matrix_to_tf_pose(T_matrix)
            tf_transforms.append(t)

        self._broadcaster.sendTransform(tf_transforms)
        self._ros_node.get_logger().info(f"[Sdf2Tf_Publisher] Published {len(tf_transforms)} static transforms.")

    # #}

    # #{ _matrix_to_tf_pose(self, T_W_Sensor: np.ndarray) -> Transform
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

    # #}
