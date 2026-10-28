#!/bin/bash
# Multi-Position Wind Gust Benchmark Execution Runner
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

source /opt/ros/humble/setup.bash
source install/setup.bash

pkill -9 -f 'flight_controller_node|aerodynamic_drag_node|wind_field_node' 2>/dev/null || true
sleep 1

ros2 launch uav_control flight_control.launch.py > /tmp/flight_control.log 2>&1 &
FC_PID=$!
ros2 launch uav_aerodynamics aerodynamics.launch.py > /tmp/aerodynamics.log 2>&1 &
AERO_PID=$!

sleep 3

EXIT_CODE=0
python3 benchmark_multi_gust_dob_vs_pid.py || EXIT_CODE=$?

kill -9 $FC_PID $AERO_PID 2>/dev/null || true
pkill -9 -f 'flight_controller_node|aerodynamic_drag_node|wind_field_node' 2>/dev/null || true

# Copy generated files to Windows workspace
cp -u /home/sarat_9_7/ros2_uav_ws/multi_gust_dob_vs_pid.png /mnt/d/ROS/ 2>/dev/null || true
cp -u /home/sarat_9_7/ros2_uav_ws/multi_gust_benchmark_pid.csv /mnt/d/ROS/ 2>/dev/null || true
cp -u /home/sarat_9_7/ros2_uav_ws/multi_gust_benchmark_dob.csv /mnt/d/ROS/ 2>/dev/null || true

exit $EXIT_CODE
