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
tmux new-window -t $SESSION:7 -n 'media_server'
tmux new-window -t $SESSION:8 -n 'axlaunch_server'
tmux new-window -t $SESSION:9 -n 'logging'
tmux new-window -t $SESSION:10 -n 'info_terminal'
tmux new-window -t $SESSION:11 -n 'bell_bot'
tmux new-window -t $SESSION:12 -n 'walking_group'
tmux new-window -t $SESSION:13 -n 'scheduler'
tmux new-window -t $SESSION:14 -n 'control'
tmux new-window -t $SESSION:15 -n 'logging_server'
tmux new-window -t $SESSION:16 -n 'pred_map_bags'
tmux new-window -t $SESSION:17 -n 'screen_broadcast'

tmux select-window -t $SESSION:0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roscore" C-m
tmux resize-pane -U 30
tmux select-pane -t 1
tmux send-keys "htop" C-m

tmux select-window -t $SESSION:1
tmux send-keys "DISPLAY=:0 roslaunch aaf_bringup aaf_core.launch"

tmux select-window -t $SESSION:2
tmux send-keys "DISPLAY=:0 roslaunch strands_bringup strands_robot.launch with_mux:=false"

tmux select-window -t $SESSION:3
tmux send-keys "DISPLAY=:0 roslaunch strands_bringup strands_cameras.launch head_camera:=true head_ip:=werner-left-cortex head_user:=strands chest_camera:=true chest_ip:=werner-right-cortex chest_user:=strands"

tmux select-window -t $SESSION:4
tmux send-keys "rosparam set /deployment_language german && HOST_IP=192.168.0.100 DISPLAY=:0 roslaunch aaf_bringup aaf_ui.launch mary_machine:=werner-right-cortex mary_machine_user:=strands"

tmux select-window -t $SESSION:5
tmux send-keys "DISPLAY=:0 roslaunch aaf_bringup aaf_navigation.launch map:=/opt/strands/map/y4_aaf-cropped.yaml topological_map:=aaf_y4"

tmux select-window -t $SESSION:6
tmux send-keys "DISPLAY=:0 roslaunch perception_people_launch people_tracker_robot.launch machine:=werner-left-cortex user:=strands with_wheelchair_detector:=true wheelchair_machine:=werner-left-cortex wheelchair_user=strands"

tmux select-window -t $SESSION:7
tmux send-keys "DISPLAY=:0 roslaunch aaf_bringup aaf_media_server.launch"

tmux select-window -t $SESSION:8
tmux send-keys "DISPLAY=:0 roslaunch aaf_bringup aaf_launch_server.launch"

tmux select-window -t $SESSION:9
tmux send-keys "DISPLAY=:0 roslaunch aaf_logging logging.launch mongodb_host:=werner-left-cortex"

tmux select-window -t $SESSION:10
tmux send-keys "DISPLAY=:0  roslaunch info_terminal info_terminal.launch"

tmux select-window -t $SESSION:11
tmux send-keys "DISPLAY=:0 roslaunch aaf_bellbot bellbot.launch"

tmux select-window -t $SESSION:12
tmux send-keys "DISPLAY=:0 roslaunch aaf_walking_group task_servers.launch"

tmux select-window -t $SESSION:13
tmux send-keys "ssh werner-right-cortex" C-m
tmux send-keys "DISPLAY=:0 roslaunch aaf_bringup aaf_routine.launch new_scheduler:=true calendar:=henry.strands%40hanheide.net"

tmux select-window -t $SESSION:14
tmux send-keys "DISPLAY=:0 roslaunch aaf_bringup aaf_deployment_control.launch"

tmux select-window -t $SESSION:15
tmux send-keys "ssh werner-left-cortex" C-m
tmux send-keys "rosrun aaf_logging start_stop_logging.py"

tmux select-window -t $SESSION:16
tmux send-keys "ssh werner-left-cortex" C-m
tmux send-keys "cd /storage" C-m
tmux send-keys "rosbag record --split --duration=1h /tf /scan /odom /amcl_pose /robot_pose /current_node /current_edge /map /topological_map /infoterminal/active_screen"

tmux select-window -t $SESSION:17
tmux send-keys "DISPLAY=:0 cvlc -vvv --no-audio screen:// --screen-fps 1 --sout '#transcode{vcodec=MJPG,vb=800}:standard{access=http,mux=mpjpeg,dst=:18223/}' --sout-http-mime='multipart/x-mixed-replace;boundary=--7b3cc56e5f51db803f790dad720ed50a'"

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse off
