from enum import Enum, StrEnum

class GazeboSensors(StrEnum):
    CAMERA = "camera"
    LIDAR = "gpu_lidar"
    RGBD_CAMERA = "rgbd_camera"

class AttachedSensors(Enum):
    CAMERAS = 0
    RGBD_CAMERAS = 1
    TWO_D_LIDAR = 2
    THREE_D_LIDAR = 3

class SensorTopics(Enum):
    IMAGE = 0
    DEPTH_IMAGE = 1
    CAMERA_INFO = 2
    POINTS = 3