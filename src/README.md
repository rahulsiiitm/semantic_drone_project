# Semantic-Aware Autonomous Drone Navigation

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white)
![C++](https://img.shields.io/badge/C++-17-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python&logoColor=white)
![YOLOv8](https://img.shields.io/badge/AI-YOLOv8-00FFFF?style=for-the-badge)

> **Current Status:** 🚧 Active Development (Phase 2: Simulation Integration)

## 📖 Overview
Standard autonomous drones treat all obstacles equally. A concrete wall and a soft bush are both seen as "Occupied Space."

This project implements a **Semantic-Aware Navigation Stack** for UAVs. By fusing **Real-time Computer Vision (YOLOv8)** with a **Custom ROS 2 Nav2 Costmap Plugin**, the drone enables **Semantic Traversability Analysis**:
* **Humans:** Treated as Critical Obstacles (Cost 255 + Safety Bubble).
* **Vegetation:** Treated as High-Cost but Traversable (Cost 128).
* **Walls/Static Objects:** Standard Lethal Obstacles.

This allows the drone to make intelligent, context-aware decisions—such as choosing to fly through tall grass to avoid getting too close to a human.

## System Architecture

| Component | Tech Stack | Description |
| :--- | :--- | :--- |
| **The Eyes** (Perception) | Python, YOLOv8-seg | Real-time semantic segmentation node that publishes a semantic mask topic. |
| **The Brain** (Navigation) | C++, ROS 2 Nav2 | Custom `Costmap2D` plugin that injects semantic costs into the global map. |
| **The Body** (Control) | PX4, MAVROS, Gazebo | Flight control stack and 3D physics simulation. |

## 📂 Project Structure
```bash
semantic_drone_project/
├── semantic_nav_pkg/        # [C++] The Custom Nav2 Plugin
│   ├── include/             # Header files for the costmap layer
│   ├── src/                 # Core logic for costmap inflation
│   └── nav2_plugins.xml     # Plugin registration for ROS 2
├── semantic_vision_pkg/     # [Python] The AI Perception Node
│   ├── semantic_vision_pkg/ # YOLOv8 Inference logic
│   └── resource/            # Markers
└── simulation/              # Custom Gazebo Worlds & Models
```

##  Installation

### Prerequisites
- **OS:** Ubuntu 22.04 LTS (Jammy Jellyfish)
- **Middleware:** ROS 2 Humble
- **Simulator:** PX4 Autopilot + Gazebo Garden
- **Hardware:** NVIDIA GPU recommended for YOLOv8 (CUDA)

### Build Instructions

**1. Clone the Repository:**

```bash
cd ~/ros2_ws/src
git clone https://github.com/rahulsiiitm/semantic_drone_project.git
```

**2. Install Dependencies:**

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

**3. Build the Stack:**

```bash
colcon build --symlink-install
```

##  Usage (Work in Progress)

**1. Launch the Simulation**

```bash
# Starts Gazebo with the Iris drone
make px4_sitl gazebo
```

**2. Start the Perception Node**

```bash
# Starts the YOLOv8 camera stream
ros2 run semantic_vision_pkg vision_node
```

**3. Start the Navigation Stack**

```bash
# Launches Nav2 with the custom semantic plugin loaded
ros2 launch semantic_nav_pkg navigation.launch.py
```

## Deep Dive: The Custom Costmap Plugin

The core innovation lives in `semantic_layer.cpp`. Unlike standard static layers, this plugin subscribes to the `/semantic_mask` topic and updates the costmap dynamically:

```cpp
// Logic Snippet from semantic_layer.cpp
if (pixel_class == PERSON) {
    master_array[index] = LETHAL_OBSTACLE; // Force Re-plan
} 
else if (pixel_class == BUSH) {
    master_array[index] = HIGH_COST;       // Traversable if necessary
}
```

## Roadmap

- [x] Phase 1: Core C++ Plugin & Python AI Node Architecture (Completed)
- [x] Phase 2: Integration with PX4 SITL & Gazebo (In Progress)
- [ ] Phase 3: "Safety Bubble" Visualization in RViz
- [ ] Phase 4: Real-world testing with MAVROS

## 🤝 Contributors

**Rahul Sharma** - Lead Developer - IIIT Manipur