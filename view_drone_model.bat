@echo off
title UAV Quadrotor 3D Model Inspection
cls
echo ======================================================================
echo   Inspecting 3D Quadrotor Drone Model & Rotor Articulations in RViz2
echo ======================================================================
echo.
wsl -d Ubuntu-22.04 bash -c "source /opt/ros/humble/setup.bash && source /home/sarat_9_7/ros2_uav_ws/install/setup.bash && ros2 launch uav_description display.launch.py"
pause
