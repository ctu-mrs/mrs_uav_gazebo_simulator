#!/usr/bin/python3
import pytest
import os
from ament_index_python.packages import get_package_share_directory
from xml.dom import minidom

from utils.jinja_loader import JinjaLoader
from mrs_uav_gazebo_simulator.utils.spawner_types import *
from utils.sensor_tag_checks import check_required_rangefinder_tags
from utils.common_utils import *

RANGEFINDERS = "mrs_robots_description/sdf/components/rangefinder/"
resource_paths = [os.path.join(get_package_share_directory('mrs_uav_gazebo_simulator'), 'models')]
loader = JinjaLoader(resource_paths)


# #{ collect_macros()
def collect_macros():
    items = []
    temp_to_macros = loader.get_template_to_macros(RANGEFINDERS)
    for template, macros in temp_to_macros.items():
        for macro in macros:
            if "template" not in macro:
                items.append((template, macro))
    return items


# #}


# #{ render_lidar_sdf(loader, template, macro_name)
def render_rangefinder_sdf(loader, template, macro_name):
    rangefinder_sdf = loader.render_macro_file(
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
    assert rangefinder_sdf.strip(), "rendered empty"
    return f"<model>{rangefinder_sdf}</model>"


# #}


@pytest.mark.parametrize("template,macro", collect_macros())
def test_rangefinder_macro(template, macro):
    """
    Verify template rendering, sensor plugin arguments, and whether the link pose is specified.
    """
    rangefinder_sdf = render_rangefinder_sdf(loader, template, macro)

    lidar_xml = minidom.parseString(rangefinder_sdf)

    # Check the sensor plugin
    sensor_blocks = lidar_xml.getElementsByTagName('sensor')
    for sensor in sensor_blocks:
        sensor_type = sensor.getAttribute('type')
        if sensor_type == GazeboSensors.LIDAR:
            check_required_rangefinder_tags(sensor)

    # Check the link pose
    link_blocks = lidar_xml.getElementsByTagName('link')
    for link in link_blocks:
        pose_str = get_elem_by_tag_name(link, 'pose')
        if not check_str_to_pose(pose_str):
            raise AssertionError(f"The <pose> tag should have 6 elements.")
