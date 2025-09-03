# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
#
# Modified version - Gazebo removed for real hardware deployment
#
# Build this Dockerfile by running the following commands:
#
#     $ cd /path/to/your/jetbot_ros
#     $ docker/build.sh
#
# Also you should set your docker default-runtime to nvidia:
#     https://github.com/dusty-nv/jetson-containers#docker-default-runtime
#

ARG BASE_IMAGE=dustynv/ros:foxy-pytorch-l4t-r32.5.0
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-c"] 
ENV SHELL /bin/bash

ENV DEBIAN_FRONTEND=noninteractive
ARG MAKEFLAGS=-j$(nproc)
ENV LANG=en_US.UTF-8 
ENV PYTHONIOENCODING=utf-8
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

WORKDIR /tmp

#
# Install essential development tools (removed Gazebo)
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		  nano \
		  vim \
		  git \
		  wget \
		  curl \
		  build-essential \
		  python3-pip \
		  python3-dev \
		  libjpeg-dev \
		  zlib1g-dev \
		  libfreetype6-dev \
		  liblcms2-dev \
		  libopenjp2-7-dev \
		  libtiff5-dev \
		  libffi-dev \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

#
# Install Python packages for hardware control
#
RUN pip3 install --no-cache-dir \
    Adafruit-MotorHAT \
    Adafruit-SSD1306 \
    adafruit-circuitpython-pca9685 \
    adafruit-circuitpython-motor \
    pyserial \
    sparkfun-qwiic \
    RPi.GPIO \
    board \
    busio \
    --verbose

#
# Install additional packages for dual camera and navigation
#
RUN pip3 install --no-cache-dir \
    opencv-python \
    numpy \
    scipy \
    scikit-image \
    matplotlib \
    --verbose

#
# Environment setup (removed Gazebo variables)
#   
ENV WORKSPACE_ROOT=/workspace
ENV JETBOT_ROOT=${WORKSPACE_ROOT}/src/jetbot_ros
ARG ROS_ENVIRONMENT=${ROS_ROOT}/install/setup.bash

# Setup workspace
WORKDIR ${WORKSPACE_ROOT}
RUN mkdir -p ${WORKSPACE_ROOT}/src

COPY scripts/setup_workspace.sh ${WORKSPACE_ROOT}/setup_workspace.sh
ENV PYTHONPATH="${JETBOT_ROOT}:${PYTHONPATH}"

#
# ros_deep_learning package (keep for camera functionality)
#
RUN source ${ROS_ENVIRONMENT} && \
    cd ${WORKSPACE_ROOT}/src && \
    git clone https://github.com/dusty-nv/ros_deep_learning && \
    cd ../ && \
    colcon build --symlink-install --event-handlers console_direct+

#
# Install additional ROS2 packages for navigation
#
RUN source ${ROS_ENVIRONMENT} && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-simple-commander \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-geometry \
    ros-${ROS_DISTRO}-stereo-image-proc \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

#
# Build project (removed gazebo plugin build)
#
COPY jetbot_ros ${JETBOT_ROOT}/jetbot_ros
COPY launch ${JETBOT_ROOT}/launch
COPY resource ${JETBOT_ROOT}/resource

COPY package.xml ${JETBOT_ROOT}
COPY setup.py ${JETBOT_ROOT}
COPY setup.cfg ${JETBOT_ROOT}

# Build the main project
RUN source ${ROS_ENVIRONMENT} && \
    cd ${WORKSPACE_ROOT} && \
    colcon build --symlink-install --event-handlers console_direct+

#
# Setup entrypoint
#
COPY scripts/ros_entrypoint.sh /ros_entrypoint.sh

RUN sed -i \
    's/ros_env_setup="\/opt\/ros\/$ROS_DISTRO\/setup.bash"/ros_env_setup="${ROS_ROOT}\/install\/setup.bash"/g' \
    /ros_entrypoint.sh && \
    cat /ros_entrypoint.sh

RUN echo 'source ${ROS_ROOT}/install/setup.bash' >> /root/.bashrc && \
    echo 'source ${WORKSPACE_ROOT}/install/local_setup.bash' >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]