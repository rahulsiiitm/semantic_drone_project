#!/bin/bash

CONTAINER_ID=$(docker ps -q -f ancestor=drone_ros2_env)

echo "Installing ROS 2 Image Viewer (if not already installed)..."
docker exec -u root $CONTAINER_ID bash -c "apt-get update && apt-get install -y ros-humble-rqt-image-view"

echo "Opening the AI Video Feed..."
docker exec -it $CONTAINER_ID bash -c "source /opt/ros/humble/setup.bash && ros2 run rqt_image_view rqt_image_view"
