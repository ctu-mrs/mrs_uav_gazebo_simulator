#!/usr/bin/python3
import pytest
import os
from ament_index_python.packages import get_package_share_directory
from utils.jinja_loader import JinjaLoader
from mrs_uav_gazebo_simulator.utils.spawner_enums import *
from xml.dom import minidom

CAMERAS = "mrs_robots_description/sdf/components/camera/"


def get_sensor_topic_from_tag_name(sensor, tag_name):
    topic = sensor.getElementsByTagName(tag_name)
    if topic:
        topic = '/' + topic[0].firstChild.data
    return topic


def replace_last_topic_segment(topic: str, new_last: str) -> str:
    parts = topic.split('/')
    parts[-1] = new_last
    return '/'.join(parts)


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


def get_rgb_camera_topics_from_xml(camera_sensor) -> None:
    gz_camera_info_topic = get_sensor_topic_from_tag_name(camera_sensor, SdfTopicTags.GZ_CAMERA_INFO)
    ros_camera_info_topic = get_sensor_topic_from_tag_name(camera_sensor, SdfTopicTags.ROS_CAMERA_INFO)
    ros_color_image_topic = get_sensor_topic_from_tag_name(camera_sensor, SdfTopicTags.ROS_COLOR_IMAGE)
    sdf_topic_tag = '/' + camera_sensor.getElementsByTagName('topic')[0].firstChild.data

    if not gz_camera_info_topic:
        assert "Missing <gz_camera_info_topic> tag."
    if not ros_camera_info_topic:
        assert "Missing <ros_camera_info_topic> tag."
    if not ros_color_image_topic:
        assert "Missing <ros_color_image_topic> tag."
    if not sdf_topic_tag:
        assert "Missing <topic> tag."

    return sdf_topic_tag, CameraRosGzBridge(
        image_topic=ros_color_image_topic,
        ros_info_topic=ros_camera_info_topic,
        gz_info_topic=gz_camera_info_topic,
    )


def check_rgb_naming_convention(sdf_tag_topic: str, custom_topics: CameraRosGzBridge):
    assert sdf_tag_topic == custom_topics.image_topic, "Gazebo image topic is not equal to ROS image topic."

    gz_camera_info = replace_last_topic_segment(sdf_tag_topic, "camera_info")
    assert gz_camera_info == custom_topics.gz_info_topic, "ROS_GZ_BRIDGE: CameraInfo Gazebo is incorrect."


def get_depth_camera_topics_from_xml(camera_sensor) -> None:
    gz_camera_info_topic = get_sensor_topic_from_tag_name(camera_sensor, SdfTopicTags.GZ_CAMERA_INFO)
    ros_camera_info_topic = get_sensor_topic_from_tag_name(camera_sensor, SdfTopicTags.ROS_CAMERA_INFO)
    ros_depth_image_topic = get_sensor_topic_from_tag_name(camera_sensor, SdfTopicTags.ROS_DEPTH_IMAGE)
    sdf_topic_tag = '/' + camera_sensor.getElementsByTagName('topic')[0].firstChild.data
    gz_pointcloud_topic = get_sensor_topic_from_tag_name(camera_sensor, SdfTopicTags.GZ_POINTCLOUD)
    ros_pointcloud_topic = get_sensor_topic_from_tag_name(camera_sensor, SdfTopicTags.ROS_POINTCLOUD)

    if not gz_camera_info_topic:
        assert "Missing <gz_camera_info_topic> tag."
    if not ros_camera_info_topic:
        assert "Missing <ros_camera_info_topic> tag."
    if not ros_depth_image_topic:
        assert "Missing <ros_depth_image_topic> tag."
    if not sdf_topic_tag:
        assert "Missing <topic> tag."
    if not gz_pointcloud_topic:
        assert "Missing <gz_pointcloud_topic> tag."
    if not ros_pointcloud_topic:
        assert "Missing <ros_pointcloud_topic> tag."

    return sdf_topic_tag, DepthCameraRosGzBridge(image_topic=ros_depth_image_topic,
                                                 ros_info_topic=ros_camera_info_topic,
                                                 gz_info_topic=gz_camera_info_topic,
                                                 ros_points_topic=ros_pointcloud_topic,
                                                 gz_points_topic=gz_pointcloud_topic)


def check_depth_naming_convention(sdf_tag_topic: str, custom_topics: DepthCameraRosGzBridge):
    assert sdf_tag_topic == custom_topics.image_topic, "Gazebo image topic is not equal to ROS image topic."

    gz_camera_info = replace_last_topic_segment(sdf_tag_topic, "camera_info")
    assert gz_camera_info == custom_topics.gz_info_topic, "ROS_GZ_BRIDGE: Gazebo CameraInfo topic is incorrect."


def get_rgbd_camera_topics_from_xml(camera_sensor) -> None:
    gz_camera_info_topic = get_sensor_topic_from_tag_name(camera_sensor, SdfTopicTags.GZ_CAMERA_INFO)
    gz_pointcloud_topic = get_sensor_topic_from_tag_name(camera_sensor, SdfTopicTags.GZ_POINTCLOUD)
    ros_camera_info_topic = get_sensor_topic_from_tag_name(camera_sensor, SdfTopicTags.ROS_CAMERA_INFO)
    ros_color_image_topic = get_sensor_topic_from_tag_name(camera_sensor, SdfTopicTags.ROS_COLOR_IMAGE)
    ros_depth_image_topic = get_sensor_topic_from_tag_name(camera_sensor, SdfTopicTags.ROS_DEPTH_IMAGE)
    ros_pointcloud_topic = get_sensor_topic_from_tag_name(camera_sensor, SdfTopicTags.ROS_POINTCLOUD)
    sdf_topic_tag = '/' + camera_sensor.getElementsByTagName('topic')[0].firstChild.data

    if not gz_camera_info_topic:
        assert "Missing <gz_camera_info_topic> tag."
    if not ros_camera_info_topic:
        assert "Missing <ros_camera_info_topic> tag."
    if not ros_depth_image_topic:
        assert "Missing <ros_depth_image_topic> tag."
    if not ros_color_image_topic:
        assert "Missing <ros_color_image_topic> tag."
    if not sdf_topic_tag:
        assert "Missing <topic> tag."
    if not gz_pointcloud_topic:
        assert "Missing <gz_pointcloud_topic> tag."
    if not ros_pointcloud_topic:
        assert "Missing <ros_pointcloud_topic> tag."

    return sdf_topic_tag, RgbdCameraRosGzBridge(rgb_image_topic=ros_color_image_topic,
                                                depth_image_topic=ros_depth_image_topic,
                                                ros_info_topic=ros_camera_info_topic,
                                                gz_info_topic=gz_camera_info_topic,
                                                ros_points_topic=ros_pointcloud_topic,
                                                gz_points_topic=gz_pointcloud_topic)


def check_rgbd_naming_convention(sdf_tag_topic: str, custom_topics: RgbdCameraRosGzBridge):
    gz_camera_info = sdf_tag_topic + "/camera_info"
    gz_depth_image = sdf_tag_topic + "/depth_image"
    gz_image = sdf_tag_topic + "/image"
    gz_points = sdf_tag_topic + "/points"

    assert gz_camera_info == custom_topics.gz_info_topic, "ROS_GZ_BRIDGE: Gazebo CameraInfo topic is incorrect."
    assert gz_image == custom_topics.rgb_image_topic, "ROS_GZ_BRIDGE: Gazebo RGB Image topic is incorrect."
    assert gz_depth_image == custom_topics.depth_image_topic, "ROS_GZ_BRIDGE: Gazebo Depth Image topic is incorrect."
    assert gz_points == custom_topics.gz_points_topic, "ROS_GZ_BRIDGE: Gazebo Points topic is incorrect."


def test_camera_topics_names():
    resource_paths = [os.path.join(get_package_share_directory('mrs_uav_gazebo_simulator'), 'models')]
    loader = JinjaLoader(resource_paths)

    failures = []

    temp_to_macros = loader.get_template_to_macros(CAMERAS)
    for template, macros in temp_to_macros.items():
        macro_name = macros[0]
        try:
            camera_sdf = get_camera_sdf(loader, template, macro_name)
            camera_xml = minidom.parseString(camera_sdf)
            sensor_blocks = camera_xml.getElementsByTagName('sensor')
            for sensor in sensor_blocks:
                sensor_type = sensor.getAttribute('type')
                if sensor_type == GazeboSensors.CAMERA:
                    sdf_tag_topic, custom_topics = get_rgb_camera_topics_from_xml(sensor)
                    check_rgb_naming_convention(sdf_tag_topic, custom_topics)
                if sensor_type == GazeboSensors.DEPTH_CAMERA:
                    sdf_tag_topic, custom_topics = get_depth_camera_topics_from_xml(sensor)
                    check_depth_naming_convention(sdf_tag_topic, custom_topics)
                if sensor_type == GazeboSensors.RGBD_CAMERA:
                    sdf_tag_topic, custom_topics = get_rgbd_camera_topics_from_xml(sensor)
                    check_rgbd_naming_convention(sdf_tag_topic, custom_topics)

        except Exception as e:
            failures.append((template, macro_name, str(e)))

    if failures:
        lines = [f"{t} :: {m} -> {err}" for (t, m, err) in failures]
        pytest.fail("Some macros failed:\n" + "\n".join(lines), pytrace=False)
