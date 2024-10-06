 docker run  -it --rm \
        --env DISPLAY=$DISPLAY \
        --network host \
        --device=/dev/input/js0 \
        --name stroam_robot \
        --volume ./ros2_ws:/stroam_remote/ros2_ws \
        stroam_remote