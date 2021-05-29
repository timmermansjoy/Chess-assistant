#!/bin/sh

# Set Session Name
SESSION="chessAI"
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

# Only create tmux session if it doesn't already exist
if [ "$SESSIONEXISTS" = "" ]
then
    # Start New Session with our name
    tmux new-session -d -s $SESSION

    # start window system
    tmux new-window -t $SESSION:1 -n 'roscore'
    tmux send-keys -t 'roscore' 'roscore' C-m

    # start window system
    tmux new-window -t $SESSION:2 -n 'vision'
    tmux send-keys -t 'vision' 'cd ~/Projects/catkin_ws/src/RP2021G02/src/abstraction/scripts' C-m
    tmux send-keys -t 'vision' 'rosrun abstraction abstraction.py' C-m

      # start window system
    tmux new-window -t $SESSION:3 -n 'rosbag'
    tmux send-keys -t 'rosbag' 'cd ~/Data/bags' C-m
    tmux send-keys -t 'rosbag' 'rosbag play -l demo.bag'

      # start window system
    tmux new-window -t $SESSION:4 -n 'gui'
    tmux send-keys -t 'gui' 'cd ~/Projects/catkin_ws/src/RP2021G02' C-m
    tmux send-keys -t 'gui' 'rosrun abstraction window.py'

    tmux join-pane -v -s 2 -t 1
    tmux join-pane -h -s 3 -t 1

fi

# Attach Session, on the Main window
tmux attach-session -t $SESSION:1
