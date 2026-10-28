#!/bin/bash
killall -9 gzserver gzclient ros2 2>/dev/null || true
pkill -9 -f 'flight_controller_node' 2>/dev/null || true
pkill -9 -f 'wind_field_node' 2>/dev/null || true
pkill -9 -f 'aerodynamic_drag_node' 2>/dev/null || true
pkill -9 -f 'uav_teleop' 2>/dev/null || true

rsync -av /mnt/d/ROS/src/ /home/sarat_9_7/ros2_uav_ws/src/ > /dev/null
source /opt/ros/humble/setup.bash
source /home/sarat_9_7/ros2_uav_ws/install/setup.bash

echo "======================================================================"
echo "        LAUNCHING UAV COMPLETE SYSTEM (ALL NODES IN 1 COMMAND)        "
echo "======================================================================"
echo "1. 3D Model in RViz2"
echo "2. 6-DOF Flight Controller and 500 Hz Physics"
echo "3. Disturbance Observer (DOB Zero-Drift Rejection)"
echo "4. Dynamic Wind Field and 3D Aerodynamic Drag"
echo "5. Autonomous Trajectory Generator"
echo "6. Dedicated Pilot Remote Controller GUI"
echo "7. Live Tuning and Aerodynamics Studio Dashboard"
echo "======================================================================"

ros2 launch uav_control uav_complete_system.launch.py "$@"
