#!/bin/bash
# Headless test launcher for the ArduPilot UAV simulation session.
# Same cleanup as start_simulation.sh, but starts session_headless.yml
# (gz headless, no rviz, no auto takeoff).
unset AMENT_TRACE_SETUP_FILES COLCON_TRACE
source /home/nekovfra/workspaces/claude_ws/install/setup.bash >/dev/null 2>&1
tmux -L mrs kill-server >/dev/null 2>&1 || true
sleep 1
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "mavros_node" 2>/dev/null
pkill -9 -f "mrs_drone_spawner" 2>/dev/null
pkill -9 -f "build/sitl" 2>/dev/null
pkill -9 -f "component_container" 2>/dev/null
pkill -9 -f "rmw_zenohd" 2>/dev/null
sleep 2
cd "$(dirname "$(readlink -f "$0")")"
tmuxinator start -p ./session_headless.yml --no-attach
