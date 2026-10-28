#!/bin/bash
set -e

# Kill any existing processes
killall -9 gzserver gzclient 2>/dev/null || true
pkill -9 -f 'flight_controller_node' 2>/dev/null || true
pkill -9 -f 'wind_field_node' 2>/dev/null || true
pkill -9 -f 'aerodynamic_drag_node' 2>/dev/null || true
pkill -9 -f 'uav_teleop' 2>/dev/null || true
pkill -9 -f 'robot_state_publisher' 2>/dev/null || true

# Sync workspace sources from D:/ROS/src
rsync -av /mnt/d/ROS/src/ /home/sarat_9_7/ros2_uav_ws/src/ > /dev/null

# Source ROS 2 Humble and workspace
source /opt/ros/humble/setup.bash
source /home/sarat_9_7/ros2_uav_ws/install/setup.bash

echo "Starting core flight and aerodynamics nodes..."
python3 /home/sarat_9_7/ros2_uav_ws/install/uav_control/lib/uav_control/flight_controller_node > /tmp/flight_controller.log 2>&1 &
P1=$!
python3 /home/sarat_9_7/ros2_uav_ws/install/uav_aerodynamics/lib/uav_aerodynamics/wind_field_node > /tmp/wind_field.log 2>&1 &
P2=$!
python3 /home/sarat_9_7/ros2_uav_ws/install/uav_aerodynamics/lib/uav_aerodynamics/aerodynamic_drag_node > /tmp/aero_drag.log 2>&1 &
P3=$!

cleanup() {
    echo "Cleaning up test nodes..."
    kill -9 $P1 $P2 $P3 2>/dev/null || true
    pkill -9 -f 'flight_controller_node' 2>/dev/null || true
    pkill -9 -f 'wind_field_node' 2>/dev/null || true
    pkill -9 -f 'aerodynamic_drag_node' 2>/dev/null || true
}
trap cleanup EXIT

echo "Waiting for ROS 2 nodes to initialize..."
sleep 2

echo "Running Rigorous Disturbance Observer & Flight Lifecycle Test..."
python3 /mnt/d/ROS/test_gust_and_dob_rigorous.py
TEST_RES=$?

if [ $TEST_RES -eq 0 ]; then
    echo "Verification Complete: ALL TESTS PASSED SUCCESSFULLY!"
else
    echo "Verification Failed with exit code $TEST_RES"
fi

exit $TEST_RES
