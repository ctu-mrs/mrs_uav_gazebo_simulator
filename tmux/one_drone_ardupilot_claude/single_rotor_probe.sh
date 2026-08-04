#!/bin/bash
M=$1
echo "== initial =="
timeout 6 gz topic -e -t /world/default/model/uav1/link/base_link/sensor/imu_sensor/imu -n 1 2>/dev/null | grep -A2 "linear_acceleration" | tail -2
timeout 8 gz topic -e -t /world/default/model/uav1/joint_state -n 1 2>/dev/null | grep -E "name: \"prop|velocity" | head -8
echo "== force 8 N.m on $M for 6s =="
timeout 6 gz topic -t /model/uav1/joint/$M/cmd_force -m gz.msgs.Double -p "data: 8.0" -r 50 >/dev/null 2>&1 &
sleep 5
echo "== joint velocities =="
timeout 6 gz topic -e -t /world/default/model/uav1/joint_state -n 1 2>/dev/null | grep -E "name: \"prop|velocity" | head -8
echo "== accel =="
timeout 6 gz topic -e -t /world/default/model/uav1/link/base_link/sensor/imu_sensor/imu -n 1 2>/dev/null | grep -A2 "linear_acceleration" | tail -2
echo "== pose =="
timeout 6 gz topic -e -t /world/default/pose/info -n 1 2>/dev/null | grep -A10 '"uav1"' | grep -A6 position | head -9
