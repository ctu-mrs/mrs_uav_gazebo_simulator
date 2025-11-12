#!/usr/bin/python3
import pytest
import os
from ament_index_python.packages import get_package_share_directory
from utils.jinja_loader import JinjaLoader

DRONES = "mrs_robots_description/sdf/drones/"


# Check syntax errors in the drones' sdf.jinja files.
def test_all_drone_macros():
    resource_paths = [os.path.join(get_package_share_directory('mrs_uav_gazebo_simulator'), 'models')]
    loader = JinjaLoader(resource_paths)
    drones_templates = loader.get_templates_from_group(DRONES)

    failures = []
    for drone_templ in drones_templates:
        try:
            ctx = {}
            ctx["spawner_args"] = {"name": "uav1"}
            loader.render_drone_file(drone_templ, **ctx)

        except Exception as e:
            failures.append((drone_templ, str(e)))

    if failures:
        lines = [f"{t} -> {err}" for (t, err) in failures]
        pytest.fail("Some macros failed:\n" + "\n".join(lines), pytrace=False)
