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

echo "---------------------------------------------------"
echo "Micro XRCE-DDS Agent installed successfully!"
echo "To start the bridge, open a new terminal in the container and run:"
echo "MicroXRCEAgent udp4 -p 8888"
echo "---------------------------------------------------"
