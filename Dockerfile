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
    && rm -rf /var/lib/apt/lists/*

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
