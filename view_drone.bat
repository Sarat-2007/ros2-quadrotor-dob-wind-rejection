@echo off
title Quadrotor UAV 3D Visualization
echo ========================================================
echo   Launching Quadrotor Drone Model in RViz2 (ROS 2 Humble)
echo ========================================================
echo Opening 3D Robot Model and Joint Controls...
wsl -d Ubuntu-22.04 bash -c "source /opt/ros/humble/setup.bash && source /home/sarat_9_7/ros2_uav_ws/install/setup.bash && ros2 launch uav_description display.launch.py"
pause
