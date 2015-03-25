#!/bin/bash
SESSION=$USER
tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'mongodb_store'
tmux new-window -t $SESSION:2 -n 'simulation'
tmux new-window -t $SESSION:3 -n 'navigation'
tmux new-window -t $SESSION:4 -n 'scheduler'
tmux new-window -t $SESSION:5 -n 'bellbot_action_server'
tmux new-window -t $SESSION:6 -n 'bellbot_feedback'
tmux new-window -t $SESSION:7 -n 'rosbridge_server'
tmux new-window -t $SESSION:8 -n 'strands_webserver'
tmux new-window -t $SESSION:9 -n 'bellbot_gui'
tmux new-window -t $SESSION:10 -n 'rviz'
tmux select-window -t $SESSION:0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roscore" C-m
tmux resize-pane -U 30
tmux select-pane -t 1
tmux send-keys "htop" C-m
tmux select-window -t $SESSION:1
tmux send-keys "roslaunch mongodb_store mongodb_store.launch db_path:=/opt/strands/mongodb_store"
tmux select-window -t $SESSION:2
tmux send-keys "roslaunch strands_morse aaf_sim_morse.launch"
tmux select-window -t $SESSION:3
tmux send-keys "roslaunch aaf_simulation aaf_navigation.launch dataset:=aaf_bellbot"
tmux select-window -t $SESSION:4
tmux send-keys "roslaunch task_executor task-scheduler.launch"
tmux select-window -t $SESSION:5
tmux send-keys "roslaunch bellbot_action_server bellbot_action_server.launch"
tmux select-window -t $SESSION:6
tmux send-keys "rosrun bellbot_gui bellbot_feedback_recorder"
tmux select-window -t $SESSION:7
tmux send-keys "roslaunch rosbridge_server rosbridge_websocket.launch"
tmux select-window -t $SESSION:8
tmux send-keys "rosrun strands_webserver strands_webserver"
tmux select-window -t $SESSION:9
tmux send-keys "rosrun bellbot_gui gui.py"
tmux select-window -t $SESSION:10
tmux send-keys "rosrun rviz rviz"
# Set default window
tmux select-window -t $SESSION:0
# Attach to session
tmux -2 attach-session -t $SESSION
tmux setw -g mode-mouse on