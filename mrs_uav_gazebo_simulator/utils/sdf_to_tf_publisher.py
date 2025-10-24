#mrs_drone_spawner Line316: sdf_content

import jinja2
import copy
import tempfile
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation as R
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

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
            raise RuntimeError(f"[MRS_DRONE_SPAWNER] base_link is not defined, cannot create tf publisher")

        

    def generate_tf_publishers(self, ros_node, sdf_xml):
        root_xml = ET.fromstring(sdf_xml)
        model_xml = root_xml.find(".//model")
        self._model_name = model_xml.attrib["name"]
        
        sensor_xml_links = self._detect_sensors(model_xml)
        sensors_tf = self._detect_sensors_transformations(sensor_xml_links)
        self._generate_static_tf_broadcasters(ros_node, sensors_tf)



    def _detect_sensors_transformations(self, sensor_links):
        sensors_Tf = {}
        for link_name, link_xml in sensor_links.items():
            pose_str = link_xml.findtext('pose')
            if (pose_str == ""):
                raise RuntimeError(f"[MRS_DRONE_SPAWNER] {link_name} has no pose, cannot create tf publisher")
            link_pose_rpy = self._str_to_pose(pose_str)
            sensors_Tf[link_name] = link_pose_rpy
        return sensors_Tf        
            

    def _str_to_pose(self, pose_str):
        x, y, z, roll, pitch, yaw = map(float, pose_str.split())
        return np.array([x, y, z, roll, pitch, yaw])

    def _detect_sensors(self, model_xml):
        sensor_xml_links = {}
        for link in model_xml.findall('.//link'):
            for sensor in link.findall('.//sensor'):
                if sensor.attrib["name"] not in self._ignored_sensors:
                    sensor_xml_links[link.attrib["name"]] = link
        return sensor_xml_links
                

    def _generate_static_tf_broadcasters(self, ros_node, sensors_tf):
        broadcaster = StaticTransformBroadcaster(ros_node)
        time_now = ros_node.get_clock().now().to_msg()

        transforms = []
        for link_name, pose in sensors_tf.items():
            t = TransformStamped()
            t.header.stamp = time_now
            t.header.frame_id = self._model_name + "/" + self._base_link
            t.child_frame_id = self._model_name + "/" + link_name

            quat = R.from_euler(seq="xyz", angles=pose[3:], degrees=False).as_quat()
            t.transform.translation.x = pose[0]
            t.transform.translation.y = pose[1]
            t.transform.translation.z = pose[2]
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = quat
            transforms.append(t)

        broadcaster.sendTransform(transforms)
        ros_node.get_logger().info(f"Published {len(transforms)} static transforms relative to {self._base_link}")


