#!/usr/bin/python3
import pytest
import os
from ament_index_python.packages import get_package_share_directory
from utils.jinja_loader import JinjaLoader
import traceback

DRONES = "mrs_robots_description/sdf/drones/"


def test_all_drone_macros():
    resource_paths = [os.path.join(get_package_share_directory('mrs_uav_gazebo_simulator'), 'models')]
    loader = JinjaLoader(resource_paths)

    failures = []

    drones_templates = loader.get_templates_from_group(DRONES)
    for drone_templ in drones_templates:
        try:
            ctx = {}
            ctx["spawner_args"] = {"name": "uav1"}
            loader.render_drone_file(drone_templ, **ctx)

        except Exception as e:
            print(f"Error while rendering {drone_templ}:")
            traceback.print_exc()
            failures.append((drone_templ, f"{type(e).__name__}: {e}"))

    if failures:
        lines = [f"{t} -> {err}" for (t, err) in failures]
        pytest.fail("Some macros failed:\n" + "\n".join(lines), pytrace=True)
