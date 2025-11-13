#!/usr/bin/python3
import pytest
import os
from ament_index_python.packages import get_package_share_directory
from utils.jinja_loader import JinjaLoader
import traceback

RANGEFINDERS = "mrs_robots_description/sdf/components/rangefinder/"


def test_all_rangefinder_macros():
    resource_paths = [os.path.join(get_package_share_directory('mrs_uav_gazebo_simulator'), 'models')]
    loader = JinjaLoader(resource_paths)

    failures = []

    temp_to_macros = loader.get_template_to_macros(RANGEFINDERS)
    for template, macros in temp_to_macros.items():
        for macro_name in macros:
            if "template" in macro_name:
                continue

            try:
                out = loader.render_macro_file(
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
                assert out.strip(), "rendered empty"
            except Exception as e:
                print(f"Error while rendering {template}::{macro_name}")
                traceback.print_exc()
                failures.append((f"{template}::{macro_name}", f"{type(e).__name__}: {e}"))

    if failures:
        lines = [f"{t} -> {err}" for (t, err) in failures]
        pytest.fail("Some macros failed:\n" + "\n".join(lines), pytrace=True)
