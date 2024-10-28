#!/bin/bash

#Script for manual remote control of Stroam

echo "Make sure ip addresses are set properly for cyclon dds file." 
read -p "Enter remote host's ip: " remote_ip
echo

cd ros2_ws/
colcon build && source install/setup.bash 

tmux new-session -d -s remote_control
tmux split-window -v
tmux send-keys -t remote_control:0.0 "ros2 run remote_controller controller" C-m
tmux send-keys -t remote_control:0.1 "python3 /stroam_remote/ros2_ws/src/stroam_camera/camera_server_view.py $remote_ip" C-m
tmux attach-session -t remote_control