FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Install essential tools and PX4 dependencies
RUN apt-get update && apt-get install -y \
    git \
    wget \
    curl \
    nano \
    sudo \
    tmux \
    python3-pip \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    python3-colcon-common-extensions \
    lsb-release \
    gnupg \
    protobuf-compiler \
    libprotobuf-dev \
    && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y \
    gz-harmonic \
    libunwind-dev \
    ros-humble-navigation2 \
    ros-humble-nav2-costmap-2d \
    ros-humble-vision-msgs \
    ros-humble-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

# Install Python AI libraries (Pin numpy < 2 and opencv < 4.9 for cv_bridge compatibility)
RUN pip3 install --no-cache-dir ultralytics "opencv-python==4.8.1.78" "numpy<2"

# Add a non-root user matching the host user to avoid permission issues with mounted volumes
ARG USERNAME=drone_user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME/workspace

# Source ROS 2 by default
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

CMD ["/bin/bash"]
