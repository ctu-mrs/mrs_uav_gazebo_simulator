from enum import StrEnum

class SensorLinkData(StrEnum):
    SENSOR_NAME = "sensor_name"
    SENSOR_TYPE = "sensor_type"
    SENSOR_OFFSET_POSE_STR = "sensor_offset_pose"
    GZ_FRAME_POSE_STR = "gz_frame_id_pose"
    GZ_FRAME_ID = "gz_frame_id"

class LinkToSensorData(StrEnum):
    LINK_POSE_STR = "link_pose_str"
    SENSORS = "sensors"