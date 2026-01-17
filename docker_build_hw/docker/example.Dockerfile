# Using official Ubuntu 22.04 image as base
FROM ubuntu:22.04

# Set timezone to Moscow
ENV TZ=Europe/Moscow
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Set environment variables for localization
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive

# Install common dependencies and utilities
RUN apt update && apt install -y    apt-utils \
                                    lsb-release \
                                    gnupg2 \
                                    net-tools \
                                    iputils-ping \
                                    build-essential \
                                    wget \
                                    unzip \
                                    curl \
                                    git \
                                    nano \
                                    ffmpeg \
                                    x11-apps \
                                    xorg-dev \
                                    libglu1-mesa-dev \
                                    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Install Python3-dev and pip
RUN apt update && apt install -y \
    python3-pip \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# Update setuptools for ROS2 Humble compatibility
RUN pip3 install --no-cache-dir setuptools==70.0.0
RUN pip3 install --no-cache-dir --upgrade pip

# Add ROS2 repository to package sources
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \
$(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble Desktop and development tools
RUN apt update && apt upgrade -y && \
    apt install -y ros-dev-tools \
    ros-humble-desktop \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 control packages
RUN apt update && apt install -y ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-publisher \
    ros-humble-joint-trajectory-controller \
    ros-humble-hardware-interface \
    ros-humble-controller-manager \
    ros-humble-control-toolbox \
    ros-humble-joint-state-publisher-gui  \
    && rm -rf /var/lib/apt/lists/*

# Add auto-completion for ros2 and colcon commands
RUN echo 'eval "$(register-python-argcomplete3 ros2)"\neval "$(register-python-argcomplete3 colcon)"' >> ~/.bashrc

# Add ROS2 Humble setup to bashrc
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

# Set working directory
WORKDIR /root

# Copy custom ROS2 workspace
COPY ros2_ws /root/ros2_ws

# Build the custom ROS2 workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /root/ros2_ws && \
    colcon build --symlink-install"

# Add custom workspace to bashrc
RUN echo 'source /root/ros2_ws/install/setup.bash' >> ~/.bashrc

# Copy entrypoint script
COPY docker/entrypoint.bash /opt/ros_entrypoint.sh
RUN chmod +x /opt/ros_entrypoint.sh

# Set entrypoint
ENTRYPOINT ["/opt/ros_entrypoint.sh"]
