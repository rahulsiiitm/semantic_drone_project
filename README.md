# Semantic Drone Autopilot

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Simulation](https://img.shields.io/badge/Simulation-PX4%20SITL-orange.svg)](https://px4.io/)
[![Hardware](https://img.shields.io/badge/Hardware-Jetson_Nano-green.svg)](https://developer.nvidia.com/embedded/jetson-nano-developer-kit)

An advanced, custom drone autopilot development project that integrates semantic vision directly into the navigation and control loop. 

Designed for deployment on a Jetson Nano microcontroller, this project utilizes **YOLOv8** for real-time semantic segmentation and object detection, feeding environmental data into a custom **ROS 2 Nav2** costmap layer (`semantic_layer`) to enable highly autonomous, context-aware flight (e.g., recognizing safe landing zones, dynamic obstacle avoidance, and path following).

## ✨ Features
* **Semantic Vision Pipeline**: Utilizes an optimized YOLOv8 segmentation model (`yolov8n-seg.pt`) to process camera feeds and publish semantic masks.
* **Context-Aware Navigation**: A custom ROS 2 Nav2 costmap plugin (`semantic_layer.cpp`) that allows the drone to make intelligent path planning decisions based on the semantic understanding of its environment.
* **PX4 SITL Integration**: Full Software In The Loop (SITL) simulation support using Gazebo and Micro XRCE-DDS to safely test autonomy algorithms before real-world deployment.
* **Edge-Optimized**: Architected specifically to run on constrained edge hardware like the NVIDIA Jetson Nano.

## 🛠️ Project Structure
```text
semantic_drone_project/
├── Dockerfile                  # Containerized Ubuntu 22.04 + ROS 2 Humble environment
├── run_docker.sh               # Helper script to build and launch the Docker container
├── yolov8n-seg.pt              # Base YOLOv8 Nano segmentation weights
├── simulation/
│   ├── models/                 # Custom Gazebo models (e.g., Quadcopter with Camera)
│   └── worlds/                 # Custom Gazebo simulation worlds
└── src/
    ├── semantic_nav_pkg/       # C++ ROS 2 package for Nav2 plugins (semantic_layer)
    └── semantic_vision_pkg/    # Python ROS 2 package for YOLO inference and publishing
```

## 🚀 Getting Started (Simulation)

To avoid dependency conflicts and ensure cross-platform compatibility (especially for developers on Arch Linux), the simulation environment is completely containerized using Docker.

### Prerequisites
* Docker installed and running
* An X11 server for GUI forwarding (for Gazebo)

### Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/semantic_drone_project.git
   cd semantic_drone_project
   ```

2. Build and launch the development container:
   ```bash
   chmod +x run_docker.sh
   ./run_docker.sh
   ```
   *Note: This script automatically handles X11 forwarding so that Gazebo simulation windows can render on your host machine.*

3. Once inside the container, build the ROS 2 workspace:
   ```bash
   colcon build
   source install/setup.bash
   ```

## 🗺️ Roadmap
- [x] Dockerized ROS 2 Humble Environment
- [ ] PX4 SITL & Gazebo Integration
- [ ] Micro XRCE-DDS Bridge Setup
- [ ] YOLOv8 Vision Node Implementation
- [ ] Nav2 Semantic Costmap Integration
- [ ] Hardware Deployment (Jetson Nano)
- [ ] Real-world Flight Testing

## 📄 License
This project is licensed under the MIT License - see the LICENSE file for details.
