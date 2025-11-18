from enum import Enum, StrEnum
from typing import TypedDict
from dataclasses import dataclass


@dataclass
class Px4MavlinkConfig:
    vehicle_base_port: int = 0
    stream_for_qgc: int = 0
    firmware_launch_delay: float = 0.0


class GazeboSensors(StrEnum):
    CAMERA = "camera"
    LIDAR = "gpu_lidar"
    RGBD_CAMERA = "rgbd_camera"
    DEPTH_CAMERA = "depth_camera"
    IMU = "imu"


class GazeboPlugins(StrEnum):
    ODOMETRY_PUBLISHER = "gz::sim::systems::OdometryPublisher"


class AttachedSensors(Enum):
    CAMERAS = 0
    RGBD_CAMERAS = 1
    TWO_D_LIDAR = 2
    THREE_D_LIDAR = 3
    DEPTH_CAMERAS = 4
    IMU = 5


class RosGzBridgeCategory(StrEnum):
    IMAGE = "image"
    DEPTH_IMAGE = "depth_image"
    CAMERA_INFO = "camera_info"
    LASER_SCAN = "laser_scan"
    POINTCLOUD = "pointcloud"
    ODOMETRY = "odometry"
    ODOMETRY_WITH_COV = "odometry_with_cov"
    IMU = "imu"


class SdfTopicTags(StrEnum):
    ROS_CAMERA_INFO = "ros_camera_info_topic"
    ROS_COLOR_IMAGE = "ros_color_image_topic"
    ROS_DEPTH_IMAGE = "ros_depth_image_topic"
    ROS_POINTCLOUD = "ros_pointcloud_topic"
    GZ_CAMERA_INFO = "gz_camera_info_topic"
    GZ_POINTCLOUD = "gz_pointcloud_topic"
    ROS_IMU = "imu_ros_topic"
    GZ_IMU = "imu_gz_topic"


class RosGzBridgeTopics(TypedDict):
    gazebo: str
    ros: str


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


@dataclass
class OdometryRosGzBridge:
    ros_odometry_topic: str
    gz_odometry_topic: str
    ros_odometry_cov_topic: str
    gz_odometry_cov_topic: str


@dataclass
class ImuRosGzBridge:
    ros_imu_topic: str
    gz_imu_topic: str
