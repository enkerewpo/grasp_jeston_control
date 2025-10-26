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
    libxrender-dev \
    bzip2

RUN apt update && apt install -y curl && \
curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/focal.noarmor.gpg | tee /usr/share/keyrings/tailscale-archive-keyring.gpg >/dev/null && \
curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/focal.tailscale-keyring.list | tee /etc/apt/sources.list.d/tailscale.list && \
apt update && \
apt install -y tailscale && \
pip3 install --upgrade pip

# Install CUDA toolkit for compiling CUDA extensions (pointnet2, knn)
RUN apt-get update && apt-get install -y \
    wget gnupg && \
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i cuda-keyring_1.1-1_all.deb && \
    apt-get update && \
    apt-get install -y cuda-toolkit-12-0 && \
    rm cuda-keyring_1.1-1_all.deb

# Install Miniconda
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh && \
    bash Miniconda3-latest-Linux-x86_64.sh -b -p /opt/miniconda3 && \
    rm Miniconda3-latest-Linux-x86_64.sh

# Add conda to PATH
ENV PATH=/opt/miniconda3/bin:$PATH

# Accept conda Terms of Service and create environment
RUN conda tos accept || true && \
    conda create -n graspnet python=3.10 -y && \
    conda clean -ya

# Set up conda environment activation in bashrc
RUN echo "source /opt/miniconda3/bin/activate graspnet" >> ~/.bashrc

# Configure conda to be more stable for large downloads
RUN conda config --set solver classic && \
    conda config --set channel_priority flexible

# Install PyTorch and dependencies (will install at runtime via install_graspnet.sh instead)
# This avoids large downloads during Docker build
# The conda environment will have PyTorch installed when you run the container

# Note: GraspNet dependencies will be installed via install_graspnet.sh after container starts

# Set CUDA environment
ENV CUDA_HOME=/usr/local/cuda-12
ENV PATH=$CUDA_HOME/bin:$PATH
ENV LD_LIBRARY_PATH=/usr/local/cuda-12/lib64

WORKDIR /root/workspace