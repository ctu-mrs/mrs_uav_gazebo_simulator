#!/usr/bin/python3
import pytest
import os
from ament_index_python.packages import get_package_share_directory
from utils.jinja_loader import JinjaLoader
from mrs_uav_gazebo_simulator.utils.spawner_types import *
from xml.dom import minidom
import traceback

CAMERAS = "mrs_robots_description/sdf/components/camera/"
resource_paths = [os.path.join(get_package_share_directory('mrs_uav_gazebo_simulator'), 'models')]
loader = JinjaLoader(resource_paths)


# #{ get_elem_by_tag_name(sensor, tag_name
def get_elem_by_tag_name(sensor, tag_name):
    elem = sensor.getElementsByTagName(tag_name)
    if not elem:
        raise AssertionError(f"Missing <{tag_name}> tag.")

    if not elem[0].firstChild or not elem[0].firstChild.data:
        raise AssertionError(f"Empty <{tag_name}> tag value.")

    return elem[0].firstChild.data


# #}


# #{ get_topic_by_tag_name(sensor, tag_name)
def get_topic_by_tag_name(sensor, tag_name):
    elem = sensor.getElementsByTagName(tag_name)
    if not elem:
        raise AssertionError(f"Missing <{tag_name}> tag.")

    if not elem[0].firstChild or not elem[0].firstChild.data:
        raise AssertionError(f"Empty <{tag_name}> tag value.")

    elem = '/' + elem[0].firstChild.data
    return elem


# #}


# #{ str_to_pose(pose_str: str)
def check_str_to_pose(pose_str: str):
    parts = pose_str.split()
    if len(parts) != 6:
        return False
    return True


# #}


# #{ replace_last_topic_segment(topic: str, new_last: str) -> str
def replace_last_topic_segment(topic: str, new_last: str) -> str:
    parts = topic.split('/')
    parts[-1] = new_last
    return '/'.join(parts)


# #}


# #{ get_camera_sdf(loader, template, macro_name)
def get_camera_sdf(loader, template, macro_name):
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


# #{ check_required_rgb_camera_tags(camera_sensor)
def check_required_rgb_camera_tags(camera_sensor):
    pose_str = get_elem_by_tag_name(camera_sensor, 'pose')
    if not check_str_to_pose(pose_str):
        raise AssertionError(f"The <pose> tag should have 6 elements.")

    get_elem_by_tag_name(camera_sensor, 'gz_frame_id')
    get_elem_by_tag_name(camera_sensor, 'update_rate')

    get_elem_by_tag_name(camera_sensor, 'topic')
    get_elem_by_tag_name(camera_sensor, 'ros_camera_info_topic')
    get_elem_by_tag_name(camera_sensor, 'ros_color_image_topic')
    get_elem_by_tag_name(camera_sensor, 'gz_camera_info_topic')

    camera = camera_sensor.getElementsByTagName('camera')[0]
    get_elem_by_tag_name(camera, 'horizontal_fov')
    image = camera.getElementsByTagName('image')[0]
    get_elem_by_tag_name(image, 'width')
    get_elem_by_tag_name(image, 'height')

    clip = camera.getElementsByTagName('clip')[0]
    get_elem_by_tag_name(clip, 'near')
    get_elem_by_tag_name(clip, 'far')

    noise = camera.getElementsByTagName('noise')[0]
    get_elem_by_tag_name(noise, 'type')
    get_elem_by_tag_name(noise, 'mean')
    get_elem_by_tag_name(noise, 'stddev')


# #}


# #{ check_required_depth_camera_tags(camera_sensor)
def check_required_depth_camera_tags(camera_sensor):
    pose_str = get_elem_by_tag_name(camera_sensor, 'pose')
    if not check_str_to_pose(pose_str):
        raise AssertionError(f"The <pose> tag should have 6 elements.")

    get_elem_by_tag_name(camera_sensor, 'gz_frame_id')
    get_elem_by_tag_name(camera_sensor, 'update_rate')

    get_elem_by_tag_name(camera_sensor, 'topic')
    get_elem_by_tag_name(camera_sensor, 'ros_camera_info_topic')
    get_elem_by_tag_name(camera_sensor, 'ros_depth_image_topic')
    get_elem_by_tag_name(camera_sensor, 'ros_pointcloud_topic')
    get_elem_by_tag_name(camera_sensor, 'gz_camera_info_topic')
    get_elem_by_tag_name(camera_sensor, 'gz_pointcloud_topic')

    camera = camera_sensor.getElementsByTagName('camera')[0]
    get_elem_by_tag_name(camera, 'horizontal_fov')
    image = camera.getElementsByTagName('image')[0]
    get_elem_by_tag_name(image, 'width')
    get_elem_by_tag_name(image, 'height')

    clip = camera.getElementsByTagName('clip')[0]
    get_elem_by_tag_name(clip, 'near')
    get_elem_by_tag_name(clip, 'far')

    noise = camera.getElementsByTagName('noise')[0]
    get_elem_by_tag_name(noise, 'type')
    get_elem_by_tag_name(noise, 'mean')
    get_elem_by_tag_name(noise, 'stddev')


# #}


# #{ check_required_rgbd_camera_tags(camera_sensor)
def check_required_rgbd_camera_tags(camera_sensor):
    pose_str = get_elem_by_tag_name(camera_sensor, 'pose')
    if not check_str_to_pose(pose_str):
        raise AssertionError(f"The <pose> tag should have 6 elements.")

    get_elem_by_tag_name(camera_sensor, 'gz_frame_id')
    get_elem_by_tag_name(camera_sensor, 'update_rate')

    get_elem_by_tag_name(camera_sensor, 'topic')
    get_elem_by_tag_name(camera_sensor, 'ros_camera_info_topic')
    get_elem_by_tag_name(camera_sensor, 'ros_color_image_topic')
    get_elem_by_tag_name(camera_sensor, 'ros_depth_image_topic')
    get_elem_by_tag_name(camera_sensor, 'ros_pointcloud_topic')
    get_elem_by_tag_name(camera_sensor, 'gz_camera_info_topic')
    get_elem_by_tag_name(camera_sensor, 'gz_pointcloud_topic')

    camera = camera_sensor.getElementsByTagName('camera')[0]
    get_elem_by_tag_name(camera, 'horizontal_fov')
    image = camera.getElementsByTagName('image')[0]
    get_elem_by_tag_name(image, 'width')
    get_elem_by_tag_name(image, 'height')
    get_elem_by_tag_name(image, 'format')

    clip = camera.getElementsByTagName('clip')[0]
    get_elem_by_tag_name(clip, 'near')
    get_elem_by_tag_name(clip, 'far')

    noise = camera.getElementsByTagName('noise')[0]
    get_elem_by_tag_name(noise, 'type')
    get_elem_by_tag_name(noise, 'mean')
    get_elem_by_tag_name(noise, 'stddev')


# #}


def collect_macros():
    items = []
    temp_to_macros = loader.get_template_to_macros(CAMERAS)
    for template, macros in temp_to_macros.items():
        for macro in macros:
            if "template" not in macro:
                items.append((template, macro))
    return items


@pytest.mark.parametrize("template,macro", collect_macros())
def test_macro(template, macro):
    """
    Verify template rendering, ROS-Gazebo topic names, and whether any arguments are missing.
    """
    camera_sdf = get_camera_sdf(loader, template, macro)
    camera_xml = minidom.parseString(camera_sdf)
    sensor_blocks = camera_xml.getElementsByTagName('sensor')
    for sensor in sensor_blocks:
        sensor_type = sensor.getAttribute('type')
        if sensor_type == GazeboSensors.CAMERA:
            sdf_tag_topic, custom_topics = get_rgb_camera_topics_from_xml(sensor)
            check_rgb_naming_convention(sdf_tag_topic, custom_topics)
            check_required_rgb_camera_tags(sensor)

        if sensor_type == GazeboSensors.DEPTH_CAMERA:
            sdf_tag_topic, custom_topics = get_depth_camera_topics_from_xml(sensor)
            check_depth_naming_convention(sdf_tag_topic, custom_topics)
            check_required_depth_camera_tags(sensor)

        if sensor_type == GazeboSensors.RGBD_CAMERA:
            sdf_tag_topic, custom_topics = get_rgbd_camera_topics_from_xml(sensor)
            check_rgbd_naming_convention(sdf_tag_topic, custom_topics)
            check_required_rgbd_camera_tags(sensor)
