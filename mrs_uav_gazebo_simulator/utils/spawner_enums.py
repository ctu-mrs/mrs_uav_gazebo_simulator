from enum import Enum, StrEnum
from typing import TypedDict
from dataclasses import dataclass

class GazeboSensors(StrEnum):
    CAMERA = "camera"
    LIDAR = "gpu_lidar"
    RGBD_CAMERA = "rgbd_camera"
    DEPTH_CAMERA = "depth_camera"

class AttachedSensors(Enum):
    CAMERAS = 0
    RGBD_CAMERAS = 1
    TWO_D_LIDAR = 2
    THREE_D_LIDAR = 3
    DEPTH_CAMERAS = 4

class SensorTopics(Enum):
    IMAGE = 0
    DEPTH_IMAGE = 1
    ROS_CAMERA_INFO = 2
    GZ_CAMERA_INFO = 3
    POINTS = 4

class SensorTopicsGzBridge(Enum):
    IMAGE = 0
    DEPTH_IMAGE = 1
    CAMERA_INFO = 2
    LASER_SCAN = 3
    POINTCLOUDS = 4

class SdfTopicTags(StrEnum):
    ROS_CAMERA_INFO = "ros_camera_info_topic"
    ROS_COLOR_IMAGE = "ros_color_image_topic"
    ROS_DEPTH_IMAGE = "ros_depth_image_topic"
    ROS_POINTCLOUD = "ros_pointcloud_topic"
    GZ_CAMERA_INFO = "gz_camera_info_topic"
    GZ_POINTCLOUD = "gz_pointcloud_topic"

class RosGzBridgeTopics(TypedDict):
    gazebo: str
    ros: str

class RosGzBridgeCategory(StrEnum):
    IMAGES = "image"
    CAMERA_INFO = "camera_info"
    POINTCLOUDS = "pointclouds"
    LASER_SCANS = "laser_scans"

@dataclass
class CameraRosGzBridge:
    image_topic: str
    ros_info_topic: str
    gz_info_topic: str     

@dataclass
class DepthCameraRosGzBridge:
    image_topic: str
    ros_info_topic: str
    gz_info_topic: str
    ros_points_topic: str
    gz_points_topic: str

@dataclass
class RgbdCameraRosGzBridge:
    rgb_image_topic: str
    depth_image_topic: str
    ros_info_topic: str
    gz_info_topic: str
    ros_points_topic: str
    gz_points_topic: str

@dataclass
class LidarRosGzBridge:
    ros_points_topic: str
    gz_points_topic: str