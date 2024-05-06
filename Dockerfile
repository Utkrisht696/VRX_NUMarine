FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

#NVIDIA ENV
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV QT_X11_NO_MITSHM 1

## ROS Enviroments
ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

RUN apt-get update && apt-get install -y lsb-release && apt-get clean all
# Install vcpkg dependencies
RUN apt install -y \
    git \
    curl \
    zip \
    unzip \
    tar \
    pkg-config \
    freeglut3-dev \
    libglew-dev \
    libglfw3-dev \
    libfftw3-dev \
    libcgal-dev \
    python3 

# ROS KEY
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt install -y wget
RUN apt-get update
# GZ-SIM KEY and GZ-SIM7 Install
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && apt-get install -y \
        gz-garden \
    && rm -rf /var/lib/apt/lists/*

# ROS INSTALLATION WITH DEPENDENCIES
RUN apt update && apt-get install -y --quiet --no-install-recommends \
    python-is-python3 \
    nodejs \
    sudo \
    npm \
    tmux \
    nano \
    less \
    xterm \
    dbus-x11 \
    docker.io \
    apt-utils \
    python3-pip \
    nlohmann-json3-dev \
    git cmake python3-vcstools curl \
    clang lldb lld wget lsb-release gnupg openssl \
    libgflags-dev 

RUN apt-get install -y --quiet --no-install-recommends \
    ros-dev-tools \
    ros-humble-desktop \
    ros-humble-sdformat-urdf \
    ros-humble-radar-msgs \
    ros-humble-ros-gzgarden \
    ros-humble-xacro \
    ros-humble-sdformat-urdf \
    ros-humble-rmw-cyclonedds-cpp

RUN apt install python3-sdformat13 ros-humble-ros-gzgarden ros-humble-xacro

RUN mkdir -p /workspaces/ros_ws/src

RUN echo "source /workspaces/ros_ws/install/setup.bash" >> ~/.bashrc
RUN echo "source /opt/ros/install/setup.bash" >> ~/.bashrc


SHELL ["/bin/bash", "-c"]
