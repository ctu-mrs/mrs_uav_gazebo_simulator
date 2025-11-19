#!/usr/bin/python3
import pytest
import os
from ament_index_python.packages import get_package_share_directory
from utils.jinja_loader import JinjaLoader

DRONES = "mrs_robots_description/sdf/drones/"
resource_paths = [os.path.join(get_package_share_directory('mrs_uav_gazebo_simulator'), 'models')]
loader = JinjaLoader(resource_paths)


# #{ collect_macros()
def collect_macros():
    items = []
    drones_templates = loader.get_templates_from_group(DRONES)

    for drone_templ in drones_templates:
        items.append(drone_templ)

    return items


# #}


@pytest.mark.parametrize("template", collect_macros())
def test_drone_macro(template):
    """
    Verify template rendering.
    """
    ctx = {}
    ctx["spawner_args"] = {"name": "uav1"}
    loader.render_drone_file(template, **ctx)
