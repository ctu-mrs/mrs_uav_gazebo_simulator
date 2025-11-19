#!/usr/bin/python3
import pytest
import os
from ament_index_python.packages import get_package_share_directory
from xml.dom import minidom

from utils.jinja_loader import JinjaLoader
from utils.camera_macro_utils import *
from mrs_uav_gazebo_simulator.utils.spawner_types import *
from utils.common_utils import *
from utils.sensor_tag_checks import check_required_depth_camera_tags, check_required_rgb_camera_tags, check_required_rgbd_camera_tags, check_required_imu_tags

CAMERAS = "mrs_robots_description/sdf/components/camera/"
resource_paths = [os.path.join(get_package_share_directory('mrs_uav_gazebo_simulator'), 'models')]
loader = JinjaLoader(resource_paths)


def collect_macros():
    items = []
    temp_to_macros = loader.get_template_to_macros(CAMERAS)
    for template, macros in temp_to_macros.items():
        for macro in macros:
            if "template" not in macro:
                items.append((template, macro))
    return items


@pytest.mark.parametrize("template,macro", collect_macros())
def test_camera_macro(template, macro):
    """
    Verify template rendering, ROS-Gazebo topic names, the link pose, and whether any arguments are missing.
    """
    camera_sdf = render_camera_sdf(loader, template, macro)
    camera_xml = minidom.parseString(camera_sdf)

    # Check the sensor plugin
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

        if sensor_type == GazeboSensors.IMU:
            check_required_imu_tags(sensor)

    # Check the link pose
    link_blocks = camera_xml.getElementsByTagName('link')
    for link in link_blocks:
        pose_str = get_elem_by_tag_name(link, 'pose')
        if not check_str_to_pose(pose_str):
            raise AssertionError(f"The <pose> tag should have 6 elements.")
