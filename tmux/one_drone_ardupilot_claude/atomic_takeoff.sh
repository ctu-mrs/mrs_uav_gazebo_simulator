#!/bin/bash
# Atomic MRS takeoff for the ardupilot session:
# enable control output -> arm -> offboard -> takeoff, no sleeps between
set -e
timeout 25 ros2 service call /uav1/control_manager/toggle_output std_srvs/srv/SetBool '{data: true}' 2>&1 | grep -E "Response|success"
(timeout 25 ros2 service call /uav1/hw_api/arming std_srvs/srv/SetBool '{data: true}' 2>&1 | grep -E "Response|success") &
(timeout 25 ros2 service call /uav1/hw_api/offboard std_srvs/srv/Trigger '{}' 2>&1 | grep -E "Response|success") &
wait
timeout 30 ros2 service call /uav1/uav_manager/takeoff std_srvs/srv/Trigger '{}' 2>&1 | grep -E "Response|success"
