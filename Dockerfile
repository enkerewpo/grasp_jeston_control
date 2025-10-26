FROM osrf/ros:humble-desktop
    
ENV ROS_DOMAIN_ID=0
ENV ROS_LOCALHOST_ONLY=0
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/shared/fast.xml
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

RUN apt update && apt install -y \
    ros-humble-rmw-fastrtps-cpp \
    python3-pip \
    python3-dev \
    build-essential \
    git \
    wget \
    libeigen3-dev \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev

RUN apt update && apt install -y curl && \
curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/focal.noarmor.gpg | tee /usr/share/keyrings/tailscale-archive-keyring.gpg >/dev/null && \
curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/focal.tailscale-keyring.list | tee /etc/apt/sources.list.d/tailscale.list && \
apt update && \
apt install -y tailscale && \
pip3 install --upgrade pip

WORKDIR /root/workspace