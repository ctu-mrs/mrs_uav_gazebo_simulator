#!/bin/bash
# Launch the ArduPilot UAV simulation tmux session (socket: mrs, session: simulation).
# Kills leftover processes from previous runs, sources the workspace and
# starts tmuxinator from the session directory (required for relative configs).
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
tmuxinator start -p ./session.yml --no-attach
