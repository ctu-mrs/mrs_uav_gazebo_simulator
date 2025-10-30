#mrs_drone_spawner Line316: sdf_content

import jinja2
import copy
import tempfile
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
    def __init__(self, base_link, ignored_sensors):
        self._model_name = ""
        self._ignored_sensors = ignored_sensors
        self._base_link = base_link
        if self._base_link is None:
            raise RuntimeError(f"[MRS_DRONE_SPAWNER] base_link (parent link) is not defined in the config file, cannot create tf publisher")

        
    def generate_tf_publishers(self, ros_node, sdf_xml):
        root_xml = ET.fromstring(sdf_xml)
        model_xml = root_xml.find(".//model")
        self._model_name = model_xml.attrib["name"]
        
        sensor_to_xml_joint = self._detect_sensors(ros_node, model_xml)
        sensors_tf = self._detect_sensors_transformations(ros_node, sensor_to_xml_joint)
        self._generate_static_tf_broadcasters(ros_node, sensors_tf)



    def _detect_sensors_transformations(self, ros_node, sensor_joints):
        sensors_Tf = {}
        for link_name, joint_xml in sensor_joints.items():
            pose_str = joint_xml.findtext('pose')
            if pose_str is None or (pose_str == ""):
                ros_node.get_logger().info(f"[MRS_DRONE_SPAWNER]  {link_name} has no pose specified in its parent joint, cannot create tf publisher")
                continue
            
            link_pose_rpy = self._str_to_pose(pose_str)
            sensors_Tf[link_name] = link_pose_rpy
        return sensors_Tf        
            

    def _str_to_pose(self, pose_str):
        parts = pose_str.split()
        if len(parts) != 6:
            raise ValueError(f"[MRS_DRONE_SPAWNER] Expected 6 elements in pose string, got {len(parts)}: {pose_str}")
        
        x, y, z, roll, pitch, yaw = map(float, parts)
        return np.array([x, y, z, roll, pitch, yaw])

    def _detect_sensors(self, ros_node, model_xml):
        sensor_xml_links = []
        for link in model_xml.findall('.//link'):
            for sensor in link.findall('.//sensor'):
                if sensor.attrib["name"] not in self._ignored_sensors:
                    sensor_xml_links.append(link.attrib["name"])
        
        sensor_to_xml_joint = {}
        for link_name in sensor_xml_links:
            for joint in model_xml.findall('.//joint'):
                child_elem = joint.find('child')
                child_name = child_elem.text.strip() 
                if child_name == link_name:
                    sensor_to_xml_joint[child_name] = joint
                    break
            if link_name not in sensor_to_xml_joint:
                ros_node.get_logger().info(f"[MRS_DRONE_SPAWNER]  {link_name} has no joint and therefore cannot create a tf publisher")

        return sensor_to_xml_joint
                

    def _generate_static_tf_broadcasters(self, ros_node, sensors_tf):
        broadcaster = StaticTransformBroadcaster(ros_node)
        time_now = ros_node.get_clock().now().to_msg()

        transforms = []
        for link_name, pose in sensors_tf.items():
            t = TransformStamped()
            t.header.stamp = time_now
            t.header.frame_id = self._model_name + "/" + self._base_link
            t.child_frame_id = self._model_name + "/" + link_name

            t.transform = self._get_sensor_pose(pose)
            transforms.append(t)

        broadcaster.sendTransform(transforms)
        ros_node.get_logger().info(f"Published {len(transforms)} static transforms relative to {self._base_link}")

    
    def _get_sensor_pose(self, pose_rpy:np.ndarray) -> Transform:
        pose = Transform()
        pose.translation.x = pose_rpy[0]
        pose.translation.y = pose_rpy[1]
        pose.translation.z = pose_rpy[3]

        quat = R.from_euler("xyz", pose_rpy[3:]).as_quat()
        pose.rotation.x = quat[0]
        pose.rotation.y = quat[1]
        pose.rotation.z = quat[2]
        pose.rotation.w = quat[3]

        return pose
