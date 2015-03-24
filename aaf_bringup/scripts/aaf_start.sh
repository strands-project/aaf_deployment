#!/bin/bash

SESSION=$USER

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'core'
tmux new-window -t $SESSION:2 -n 'robot'
tmux new-window -t $SESSION:3 -n 'cameras'
tmux new-window -t $SESSION:4 -n 'ui'
tmux new-window -t $SESSION:5 -n 'navigation'
tmux new-window -t $SESSION:6 -n 'ppl_perception'


tmux select-window -t $SESSION:0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roscore" C-m
tmux resize-pane -U 30
tmux select-pane -t 1
tmux send-keys "htop" C-m

tmux select-window -t $SESSION:1
tmux send-keys "roslaunch strands_bringup strands_core.launch db_path:=/opt/strands/aaf_datacentre"

tmux select-window -t $SESSION:2
tmux send-keys "roslaunch strands_bringup strands_robot.launch laser:=/dev/laser with_mux:=false"

tmux select-window -t $SESSION:3
tmux send-keys "roslaunch strands_bringup strands_cameras.launch head_camera:=true head_ip:=left-cortex head_user:=strands chest_camera:=true chest_ip:=right-cortex chest_user:=strands"

tmux select-window -t $SESSION:4
tmux send-keys "roslaunch strands_bringup strands_ui.launch"

tmux select-window -t $SESSION:5
tmux send-keys "roslaunch strands_bringup strands_navigation.launch map:=/opt/strands/maps/WW_GF_2015_02_22-cropped.yaml with_no_go_map:=false topological_map:=WW_GF_2015_02_22 no_go_map:=/opt/strands/maps/WW_GF_2015_02_22-cropped.yaml with_human_aware:=true"

tmux select-window -t $SESSION:6
#tmux send-keys "roslaunch strands_linda linda_people_perception.launch"
tmux send-keys "roslaunch perception_people_launch people_tracker_robot.launch machine:=left-cortex user:=strands"

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse on
