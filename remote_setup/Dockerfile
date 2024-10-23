FROM osrf/ros:humble-desktop-full

WORKDIR /stroam_remote
COPY cyclonedds.xml /tmp/cyclonedds.xml
COPY .tmux.conf /root
COPY packages /stroam_remote/packages
COPY startup /stroam_remote/startup

#Setup for cyclondds middleware
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=/tmp/cyclonedds.xml
ENV ROS_DOMAIN_ID=0

#Install packages
RUN apt-get update && apt-get install --no-install-recommends -y \
iputils-ping python3-pip ros-humble-rmw-cyclonedds-cpp tmux vim tree
RUN python3 -m pip install -r ./packages/requirements.txt
RUN pip install ./packages/xbox360controller-1.1.2/

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc 
RUN echo "source /stroam_remote/ws/install/setup.bash" >> /root/.bashrc 

CMD ["/bin/bash"] 