#!/bin/bash

# Ensure simulator is running
CONTAINER_ID=$(docker ps -q -f ancestor=drone_ros2_env)
if [ -z "$CONTAINER_ID" ]; then
    echo "ERROR: The Gazebo simulator (run_docker.sh) is not running!"
    echo "Please open a terminal and run ./run_docker.sh first."
    exit 1
fi

echo "🚀 Compiling the Autopilot Workspace..."
docker exec $CONTAINER_ID bash -c "cd ~/workspace && colcon build --packages-select semantic_nav_pkg semantic_vision_pkg"

echo "✅ Compilation Complete! Launching the Autonomous Fleet..."

echo "Cleaning up old background processes..."
docker exec $CONTAINER_ID bash -c "pkill -f MicroXRCEAgent || true; pkill -f ros_gz_bridge || true; pkill -f vision_node || true; pkill -f lidar_processor_node || true; pkill -f autopilot_node || true"
sleep 1

echo "1/4 Starting MicroXRCEAgent (Bridge to PX4)..."
docker exec -d $CONTAINER_ID bash -c "~/workspace/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent udp4 -p 8888"
sleep 2

echo "2/4 Starting ROS-Gazebo Bridge (Camera Feed)..."
docker exec -d $CONTAINER_ID bash -c "source /opt/ros/humble/setup.bash && ros2 run ros_gz_bridge parameter_bridge /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image --ros-args -r /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image:=/camera/image_raw"
sleep 2

echo "3/4 Starting YOLOv8 Semantic Vision Node..."
docker exec -d $CONTAINER_ID bash -c "source ~/workspace/install/setup.bash && ros2 run semantic_vision_pkg vision_node --ros-args -p use_webcam:=false"
sleep 2

echo "3.5/4 Starting LiDAR Obstacle Processor..."
docker exec -d $CONTAINER_ID bash -c "source ~/workspace/install/setup.bash && ros2 run semantic_nav_pkg lidar_processor_node"
sleep 2

echo "4/4 Engaging Reactive Autopilot Node! 🛫"
docker exec -it $CONTAINER_ID bash -c "source /opt/ros/humble/setup.bash && source ~/workspace/install/setup.bash && ros2 run semantic_nav_pkg autopilot_node"

echo "=========================================================="
echo "🎯 ALL SYSTEMS GO!"
echo "Switch over to your Gazebo window to watch the drone fly!"
echo "=========================================================="
