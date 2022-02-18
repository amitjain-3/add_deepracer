#!/usr/bin/env bash
echo "Downloading Python libs"
echo "[1/2] SmBus2..."
pip3 install smbus2
echo "[2/2] CVXPY..."
pip3 install cvxpy
echo "Cloning the async_web_server_cpp package"
cd ~/deepracer_ws/add_deepracer/deepracer_follow_the_leader_ws
git clone https://github.com/GT-RAIL/async_web_server_cpp.git
cd async_web_server_cpp
git checkout ros2
apt-get install -y python3-websocket
echo ""
echo "Cloning the web_video_server package"
cd ~/deepracer_ws/add_deepracer/deepracer_follow_the_leader_ws
git clone https://github.com/RobotWebTools/web_video_server.git
cd web_video_server
git fetch origin pull/111/head:ros2
git checkout ros2
echo ""
echo "Cloning the rplidar_ros package"
echo "CAREFUL! This code was saved from a left repo"
cd ~/deepracer_ws/add_deepracer/deepracer_follow_the_leader_ws
git clone https://github.com/aymwc16/rplidar_ros2.git
