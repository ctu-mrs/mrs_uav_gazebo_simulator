#!/bin/bash
# Instrumented takeoff experiment for the ArduPilot+MRS session.
# Captures, in parallel:
#   - SITL ATTITUDE + SERVO_OUTPUT_RAW + STATUSTEXT  (takeoff_watch.log)
#   - MRS->FCU attitude commands (attitude_cmd.log)
#   - MRS uav_state snapshots around the attempt (uav_state_probe.log)
# then runs atomic_takeoff.sh and samples uav_state for ~25 s.
#
# Run from a session pane: bash ./armed_takeoff_experiment.sh
cd "$(dirname "$(readlink -f "$0")")"

: > takeoff_watch.log
: > attitude_cmd.log
: > setpoint_attitude.log
: > uav_state_probe.log

pkill -f takeoff_watch.py 2>/dev/null; sleep 0.5

( python3 takeoff_watch.py 75 &>/dev/null ) &
WATCH_PID=$!

# what the MRS api actually forwards to the FCU (pre-MAVROS ENU frame)
( timeout 45 ros2 topic echo /uav1/mavros/setpoint_raw/attitude \
    2>/dev/null > setpoint_attitude.log ) &
timeout 45 ros2 topic echo /uav1/hw_api/attitude_cmd \
    2>/dev/null > attitude_cmd.log &
CMD_PID=$!

timeout 45 ros2 topic echo /uav1/mavros/setpoint_raw/attitude \
    2>/dev/null > setpoint_attitude.log &
SP_PID=$!

sleep 2
echo "=== running atomic_takeoff.sh ==="
bash ./atomic_takeoff.sh

for i in $(seq 1 8); do
  sleep 3
  echo "--- t=+$((i*3))s ---" >> uav_state_probe.log
  timeout 5 ros2 topic echo --once /uav1/estimation_manager/uav_state \
    mrs_msgs/msg/UavState --field pose.position >> uav_state_probe.log 2>&1
  timeout 5 ros2 topic echo --once /uav1/estimation_manager/uav_state \
    mrs_msgs/msg/UavState --field header.stamp >> /dev/null 2>&1
done

kill $WATCH_PID $CMD_PID $SP_PID 2>/dev/null
echo "=== experiment done; logs: takeoff_watch.log attitude_cmd.log setpoint_attitude.log uav_state_probe.log ==="
