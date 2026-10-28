#!/bin/bash
killall -9 gzserver gzclient ros2 2>/dev/null || true
pkill -9 -f 'flight_controller_node' 2>/dev/null || true
pkill -9 -f 'wind_field_node' 2>/dev/null || true
pkill -9 -f 'aerodynamic_drag_node' 2>/dev/null || true
pkill -9 -f 'uav_teleop' 2>/dev/null || true

rsync -av /mnt/d/ROS/src/ /home/sarat_9_7/ros2_uav_ws/src/ > /dev/null
source /opt/ros/humble/setup.bash
cd /home/sarat_9_7/ros2_uav_ws && colcon build --symlink-install > /dev/null
source /home/sarat_9_7/ros2_uav_ws/install/setup.bash

export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:$GAZEBO_PLUGIN_PATH
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH

echo "======================================================================"
echo "          UAV FLIGHT SIMULATION INSIDE GAZEBO 3D (ROS 2 HUMBLE)       "
echo "======================================================================"
echo "1. Gazebo 3D Physics Simulator with Quadrotor UAV"
echo "2. 6-DOF Flight Controller & Aerodynamic Disturbance Observer (DOB)"
echo "3. Rayleigh Fluid Drag & Dynamic Wind Simulation"
echo "4. Official Ground Control Station (GCS) Dashboard"
echo "======================================================================"

ros2 launch uav_control uav_gazebo.launch.py dashboard:=true "$@"
