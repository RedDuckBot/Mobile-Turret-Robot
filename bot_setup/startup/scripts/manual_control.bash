#!/bin/bash

echo "Before running Stroam in manual mode, make sure your logged in as root user. Also, set ip addresses in host's cyclone dds file and params file in Stroam's camera ROS2 pkg."
read -n 1 -p "Enter y/n if requirements met: " user_char
echo 

if [ "$user_char" == "y" ]; then
    killall pigpiod;  pigpiod

    tmux new-session -d -s manual_control_stroam
    tmux split-window -v
    tmux split-window -h
    tmux send-keys -t manual_control_stroam:0.0 "ros2 launch stroam_bringup manual_control.launch.xml" C-m #Run a command in the first pane
    tmux send-keys -t manual_control_stroam:0.1 "ros2 run stroam_camera camera_publisher --ros-args --params-file ./ros2_ws/src/stroam_camera/config/params.yaml " C-m
    tmux send-keys -t manual_control_stroam:0.2 "ros2 lifecycle set camera_publisher configure"
    tmux attach-session -t manual_control_stroam 
else
    echo "Rerun this script when requirements are met." 
fi