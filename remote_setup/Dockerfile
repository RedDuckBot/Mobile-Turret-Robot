FROM osrf/ros:humble-desktop-full

WORKDIR /stroam_remote
COPY cyclonedds.xml /tmp/cyclonedds.xml
COPY .tmux.conf /root
COPY Packages /stroam_remote/Packages

#Setup for cyclondds middleware
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=/tmp/cyclonedds.xml
ENV ROS_DOMAIN_ID=0

#Install packages
RUN apt-get update && apt-get install --no-install-recommends -y \
iputils-ping python3-pip ros-humble-rmw-cyclonedds-cpp tmux vim
RUN python3 -m pip install -r ./Packages/requirements.txt

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc 
RUN echo "source /stroam_remote/ws/install/setup.bash" >> /root/.bashrc 

CMD ["/bin/bash"] 