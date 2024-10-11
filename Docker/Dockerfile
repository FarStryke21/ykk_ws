# Use the official ROS Noetic base image
FROM osrf/ros:noetic-desktop-full

# Install NVIDIA support dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    curl \
    ca-certificates \
    gnupg \
    lsb-release \ 
    gedit net-tools iputils-ping \
    python3-pip \
    python3-wstool \
    python3-catkin-tools \
    git \
    && rm -rf /var/lib/apt/lists/*

# Add the NVIDIA package repositories
RUN curl -s -L https://nvidia.github.io/libnvidia-container/gpgkey | apt-key add - && \
    distribution=$(. /etc/os-release;echo $ID$VERSION_ID) && \
    curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Support for Space Mouse
RUN apt-get update && apt-get install -y \
    spacenavd \
    libspnav-dev \
    x11-apps

RUN spacenavd
    
# Zivid Install
RUN apt-get update && apt-get install --assume-yes \
    wget \
    ocl-icd-libopencl1

RUN mkdir -p /etc/OpenCL/vendors && \
    echo "libnvidia-Screenshot from 2024-10-09 13-47-45opencl.so.1" > /etc/OpenCL/vendors/nvidia.icd

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility

RUN wget --quiet \
    https://downloads.zivid.com/sdk/releases/2.13.1+18e79e79-1/u20/amd64/zivid_2.13.1+18e79e79-1_amd64.deb \
    https://downloads.zivid.com/sdk/releases/2.13.1+18e79e79-1/u20/amd64/zivid-studio_2.13.1+18e79e79-1_amd64.deb \
    https://downloads.zivid.com/sdk/releases/2.13.1+18e79e79-1/u20/amd64/zivid-tools_2.13.1+18e79e79-1_amd64.deb \
    https://downloads.zivid.com/sdk/releases/2.13.1+18e79e79-1/u20/amd64/zivid-genicam_2.13.1+18e79e79-1_amd64.deb

RUN apt-get update
RUN apt-get install ./*.deb --assume-yes && rm ./*.deb
# Zivid Install End

# Install CA Certificates
RUN apt-get update && apt-get install -y ca-certificates
RUN update-ca-certificates

# Install Python Packages
RUN apt-get update && apt-get install -y python3-tk

# Update pip to >=20.3
RUN pip install -U pip>=20.3

RUN pip install --ignore-installed \
    open3d

# Clean up
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

RUN apt-get install -y --fix-missing ros-${ROS_DISTRO}-moveit
# Install ROS Packages
RUN apt-get install -y \
    ros-${ROS_DISTRO}-ros-control \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-realsense2-description \
    ros-${ROS_DISTRO}-catkin \
    ros-${ROS_DISTRO}-moveit-visual-tools \
    ros-${ROS_DISTRO}-rosserial \
    ros-${ROS_DISTRO}-rosserial-arduino \
    ros-${ROS_DISTRO}-industrial-robot-status-interface \
    ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-scaled-controllers \
    ros-${ROS_DISTRO}-ur-msgs \
    ros-${ROS_DISTRO}-ur-client-library \
    ros-${ROS_DISTRO}-pass-through-controllers \
    ros-${ROS_DISTRO}-trac-ik \
    ros-${ROS_DISTRO}-rqt-joint-trajectory-controller \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-spacenav-node \
    ros-${ROS_DISTRO}-moveit-servo

# Rosdep update
RUN rosdep update

WORKDIR /home/catkin_ws

# Create a symbolic link for python3 -> python
RUN ln -s /usr/bin/python3 /usr/bin/python

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

