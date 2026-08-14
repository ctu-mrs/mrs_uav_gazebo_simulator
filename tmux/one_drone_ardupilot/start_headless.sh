#!/bin/bash

# Headless variant of start.sh: launches session_headless.yml (no gazebo GUI,
# no rviz/layout). Arming/takeoff is driven manually (e.g. atomic_takeoff.sh)
# because the autostart/takeoff race can latch the "we landed in emergency"
# failsafe.

SCRIPT=$(readlink -f $0)
SCRIPTPATH=`dirname $SCRIPT`
cd "$SCRIPTPATH"

export TMUX_SESSION_NAME=simulation
export TMUX_SOCKET_NAME=mrs

tmuxinator start -p ./session_headless.yml

# attach unless the caller asked for detached (CI/headless boxes)
if [ -z "$TMUX" ]; then
  tmux -L $TMUX_SOCKET_NAME a -t $TMUX_SESSION_NAME
else
  tmux detach-client -E "tmux -L $TMUX_SOCKET_NAME a -t $TMUX_SESSION_NAME"
fi
