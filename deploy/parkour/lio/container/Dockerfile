FROM ros:noetic-ros-base-focal@sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive ROS_MASTER_URI=http://127.0.0.1:11319 ROS_IP=127.0.0.1
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential git ca-certificates python3-dev python3-numpy \
    ros-noetic-pcl-ros ros-noetic-eigen-conversions ros-noetic-tf && \
    rm -rf /var/lib/apt/lists/*
ARG POINT_LIO_COMMIT=18ed5976d8fab2bd8a5148c26a40692bd3c0dc91
RUN mkdir -p /ws/src && cd /ws/src && \
    git clone https://github.com/unitreerobotics/point_lio_unilidar.git && \
    cd point_lio_unilidar && git checkout "$POINT_LIO_COMMIT" && \
    git rev-parse HEAD > /point_lio_revision && \
    cd /ws && source /opt/ros/noetic/setup.bash && catkin_make -j2 -DPYTHON_EXECUTABLE=/usr/bin/python3
COPY protocol.py ros_gateway.py /adapter/
COPY container/start.sh /adapter/start.sh
RUN chmod +x /adapter/start.sh && dpkg-query -W > /installed-packages.txt
ENTRYPOINT ["/adapter/start.sh"]
