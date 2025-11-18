#!/usr/bin/python3
import pytest
import os
from ament_index_python.packages import get_package_share_directory
from utils.jinja_loader import JinjaLoader
import traceback

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


@pytest.mark.parametrize("template,macro", collect_macros())
def test_rangefinder_macro(template, macro):
    out = loader.render_macro_file(
        template,
        macro,
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
    assert out.strip(), "rendered empty"
