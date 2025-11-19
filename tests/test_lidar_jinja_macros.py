#!/usr/bin/python3
import pytest
import os
from ament_index_python.packages import get_package_share_directory
from utils.jinja_loader import JinjaLoader
from xml.dom import minidom

from utils.common_utils import *
from mrs_uav_gazebo_simulator.utils.spawner_types import *
from utils.sensor_tag_checks import check_required_imu_tags, check_required_lidar_tags

LIDARS = "mrs_robots_description/sdf/components/lidar/"
resource_paths = [os.path.join(get_package_share_directory('mrs_uav_gazebo_simulator'), 'models')]
loader = JinjaLoader(resource_paths)


# #{ collect_macros()
def collect_macros():
    items = []
    temp_to_macros = loader.get_template_to_macros(LIDARS)
    for template, macros in temp_to_macros.items():
        for macro in macros:
            if "template" not in macro:
                items.append((template, macro))
    return items


# #}


# #{ render_lidar_sdf(loader, template, macro_name)
def render_lidar_sdf(loader, template, macro_name):
    lidar_sdf = loader.render_macro_file(
        template,
        macro_name,
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
    assert lidar_sdf.strip(), "rendered empty"
    return f"<model>{lidar_sdf}</model>"


# #}


@pytest.mark.parametrize("template,macro", collect_macros())
def test_lidar_macro(template, macro):
    """
    Verify template rendering, sensor plugin arguments, and whether the link pose is specified.
    """
    lidar_sdf = render_lidar_sdf(loader, template, macro)

    lidar_xml = minidom.parseString(lidar_sdf)

    # Check the sensor plugin
    sensor_blocks = lidar_xml.getElementsByTagName('sensor')
    for sensor in sensor_blocks:
        sensor_type = sensor.getAttribute('type')
        if sensor_type == GazeboSensors.LIDAR:
            check_required_lidar_tags(sensor)
        if sensor_type == GazeboSensors.IMU:
            check_required_imu_tags(sensor)

    # Check the link pose
    link_blocks = lidar_xml.getElementsByTagName('link')
    for link in link_blocks:
        pose_str = get_elem_by_tag_name(link, 'pose')
        if not check_str_to_pose(pose_str):
            raise AssertionError(f"The <pose> tag should have 6 elements.")
