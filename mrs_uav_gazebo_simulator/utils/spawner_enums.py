from enum import Enum, StrEnum

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
    CAMERA_INFO = 2
    POINTS = 3

class SensorTopicsGzBridge(Enum):
    IMAGE = 0
    DEPTH_IMAGE = 1
    CAMERA_INFO = 2
    LASER_SCAN = 3
    POINTCLOUDS = 4
