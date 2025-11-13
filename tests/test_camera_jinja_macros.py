#!/usr/bin/python3
import pytest
import os
from ament_index_python.packages import get_package_share_directory
from utils.jinja_loader import JinjaLoader

CAMERAS = "mrs_robots_description/sdf/components/camera/"


def test_all_camera_macros():
    resource_paths = [os.path.join(get_package_share_directory('mrs_uav_gazebo_simulator'), 'models')]
    loader = JinjaLoader(resource_paths)

    failures = []

    temp_to_macros = loader.get_template_to_macros(CAMERAS)
    for template, macros in temp_to_macros.items():
        macro_name = macros[0]
        if 'template_' in macro_name:
            continue
        try:
            out = loader.render_macro_file(
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
            assert out.strip(), "rendered empty"
        except Exception as e:
            failures.append((template, macro_name, str(e)))

    if failures:
        lines = [f"{t} :: {m} -> {err}" for (t, m, err) in failures]
        pytest.fail("Some macros failed:\n" + "\n".join(lines), pytrace=False)
