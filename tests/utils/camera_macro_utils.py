from mrs_uav_gazebo_simulator.utils.spawner_types import *
from utils.common_utils import *


# #{ render_camera_sdf(loader, template, macro_name)
def render_camera_sdf(loader, template, macro_name):
    camera_sdf = loader.render_macro_file(
        template,
        macro_name,
        camera_name="camera_dummy",
        parent_link="base_link",
        x=0,
        y=0,
        z=0,
        roll=0,
        pitch=0,
        yaw=0,
        mount=None,
        spawner_args={"name": "uav1"},
    )
    assert camera_sdf.strip(), "rendered empty"
    return f"<model>{camera_sdf}</model>"


# #}


# #{ get_rgb_camera_topics_from_xml(camera_sensor)
def get_rgb_camera_topics_from_xml(camera_sensor):
    gz_camera_info_topic = get_topic_by_tag_name(camera_sensor, SdfTopicTags.GZ_CAMERA_INFO)
    ros_camera_info_topic = get_topic_by_tag_name(camera_sensor, SdfTopicTags.ROS_CAMERA_INFO)
    ros_color_image_topic = get_topic_by_tag_name(camera_sensor, SdfTopicTags.ROS_COLOR_IMAGE)
    sdf_topic_tag = get_topic_by_tag_name(camera_sensor, 'topic')

    return sdf_topic_tag, CameraRosGzBridge(
        image_topic=ros_color_image_topic,
        ros_info_topic=ros_camera_info_topic,
        gz_info_topic=gz_camera_info_topic,
    )


# #}


# #{ check_rgb_naming_convention(sdf_tag_topic: str, custom_topics: CameraRosGzBridge)
def check_rgb_naming_convention(sdf_tag_topic: str, custom_topics: CameraRosGzBridge):
    assert sdf_tag_topic == custom_topics.image_topic, f"Gazebo image topic is not equal to ROS image topic."

    gz_camera_info = replace_last_topic_segment(sdf_tag_topic, "camera_info")
    assert gz_camera_info == custom_topics.gz_info_topic, f"Gazebo CameraInfo topic does not match expected."


# #}


# #{ get_depth_camera_topics_from_xml(camera_sensor)
def get_depth_camera_topics_from_xml(camera_sensor):
    gz_camera_info_topic = get_topic_by_tag_name(camera_sensor, SdfTopicTags.GZ_CAMERA_INFO)
    ros_camera_info_topic = get_topic_by_tag_name(camera_sensor, SdfTopicTags.ROS_CAMERA_INFO)
    ros_depth_image_topic = get_topic_by_tag_name(camera_sensor, SdfTopicTags.ROS_DEPTH_IMAGE)
    gz_pointcloud_topic = get_topic_by_tag_name(camera_sensor, SdfTopicTags.GZ_POINTCLOUD)
    ros_pointcloud_topic = get_topic_by_tag_name(camera_sensor, SdfTopicTags.ROS_POINTCLOUD)
    sdf_topic_tag = get_topic_by_tag_name(camera_sensor, 'topic')

    return sdf_topic_tag, DepthCameraRosGzBridge(image_topic=ros_depth_image_topic,
                                                 ros_info_topic=ros_camera_info_topic,
                                                 gz_info_topic=gz_camera_info_topic,
                                                 ros_points_topic=ros_pointcloud_topic,
                                                 gz_points_topic=gz_pointcloud_topic)


# #}


# #{ check_depth_naming_convention(sdf_tag_topic: str, custom_topics: DepthCameraRosGzBridge)
def check_depth_naming_convention(sdf_tag_topic: str, custom_topics: DepthCameraRosGzBridge):
    assert sdf_tag_topic == custom_topics.image_topic, f"Gazebo image topic is not equal to ROS image topic."

    gz_camera_info = replace_last_topic_segment(sdf_tag_topic, "camera_info")
    assert gz_camera_info == custom_topics.gz_info_topic, f"Gazebo CameraInfo topic does not match expected."


# #}


# #{ get_rgbd_camera_topics_from_xml(camera_sensor)
def get_rgbd_camera_topics_from_xml(camera_sensor):
    gz_camera_info_topic = get_topic_by_tag_name(camera_sensor, SdfTopicTags.GZ_CAMERA_INFO)
    gz_pointcloud_topic = get_topic_by_tag_name(camera_sensor, SdfTopicTags.GZ_POINTCLOUD)
    ros_camera_info_topic = get_topic_by_tag_name(camera_sensor, SdfTopicTags.ROS_CAMERA_INFO)
    ros_color_image_topic = get_topic_by_tag_name(camera_sensor, SdfTopicTags.ROS_COLOR_IMAGE)
    ros_depth_image_topic = get_topic_by_tag_name(camera_sensor, SdfTopicTags.ROS_DEPTH_IMAGE)
    ros_pointcloud_topic = get_topic_by_tag_name(camera_sensor, SdfTopicTags.ROS_POINTCLOUD)
    sdf_topic_tag = get_topic_by_tag_name(camera_sensor, 'topic')

    return sdf_topic_tag, RgbdCameraRosGzBridge(rgb_image_topic=ros_color_image_topic,
                                                depth_image_topic=ros_depth_image_topic,
                                                ros_info_topic=ros_camera_info_topic,
                                                gz_info_topic=gz_camera_info_topic,
                                                ros_points_topic=ros_pointcloud_topic,
                                                gz_points_topic=gz_pointcloud_topic)


# #}


# #{ check_rgbd_naming_convention(sdf_tag_topic: str, custom_topics: RgbdCameraRosGzBridge)
def check_rgbd_naming_convention(sdf_tag_topic: str, custom_topics: RgbdCameraRosGzBridge):
    gz_camera_info = sdf_tag_topic + "/camera_info"
    gz_depth_image = sdf_tag_topic + "/depth_image"
    gz_image = sdf_tag_topic + "/image"
    gz_points = sdf_tag_topic + "/points"

    assert gz_camera_info == custom_topics.gz_info_topic, f"Gazebo CameraInfo topic does not match expected."
    assert gz_image == custom_topics.rgb_image_topic, f"Gazebo RGB Image topic does not match expected."
    assert gz_depth_image == custom_topics.depth_image_topic, f"Gazebo Depth Image topic does not match expected."
    assert gz_points == custom_topics.gz_points_topic, f"Gazebo Pointcloud topic does not match expected."


# #}
