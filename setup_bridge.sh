#!/bin/bash

# Ensure we are in the workspace
cd /home/drone_user/workspace

echo "Setting up Micro XRCE-DDS Agent..."
if [ ! -d "Micro-XRCE-DDS-Agent" ]; then
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
fi

cd Micro-XRCE-DDS-Agent
mkdir -p build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/

echo "Downloading px4_msgs for ROS 2 workspace..."
cd /home/drone_user/workspace/src
if [ ! -d "px4_msgs" ]; then
    git clone https://github.com/PX4/px4_msgs.git
fi

cd /home/drone_user/workspace

echo "---------------------------------------------------"
echo "Micro XRCE-DDS Agent installed successfully!"
echo "To start the bridge, open a new terminal in the container and run:"
echo "MicroXRCEAgent udp4 -p 8888"
echo "---------------------------------------------------"
