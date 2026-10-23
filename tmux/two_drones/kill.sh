#!/bin/bash

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`
cd "$SCRIPTPATH"

export TMUX_SESSION_NAME=simulation
export TMUX_SOCKET_NAME=mrs

# kill the session from a new pane in it, the same as prefix+k (killTmuxSession is from mrs_uav_shell_additions)
tmux -L $TMUX_SOCKET_NAME split-window -t $TMUX_SESSION_NAME
tmux -L $TMUX_SOCKET_NAME send-keys -t $TMUX_SESSION_NAME "killTmuxSession --confirmed; exit" ENTER
