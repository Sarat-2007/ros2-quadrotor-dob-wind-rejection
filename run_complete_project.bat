@echo off
title UAV Complete Flight System - Full Autonomous, Teleop, Physics & RViz2
cls
echo ======================================================================
echo          UAV COMPLETE AUTONOMOUS & TELEOPERATED FLIGHT SYSTEM
echo                  ROS 2 HUMBLE (WSL2 Ubuntu 22.04)
echo ======================================================================
echo.
echo Starting All Components in Single Master Launch:
echo  1. 3D Quadrotor URDF Model (Robot State Publisher)
echo  2. 6-DOF Flight Controller & 500 Hz Physics Simulation
echo  3. Active Disturbance Observer (DOB / ADRC Zero-Drift Wind Cancellation)
echo  4. Dynamic Wind Field Generator (Mean + 1-Cosine Gusts + Turbulence)
echo  5. 3D Aerodynamic Drag Engine (Rayleigh Fluid Drag Law)
echo  6. Autonomous Trajectory Generator (Circle, Spline, S-Curve Waypoints)
echo  7. RViz2 3D Visualizer (Model, Spinning Propellers, 3D Flight Trail)
echo  8. Dedicated Pilot Remote Controller GUI (Takeoff/Land/Hover, D-Pad, HUD)
echo  9. Live Aerodynamics Tuning & Disturbance Observer Studio Dashboard
echo.
echo Close this window or press Ctrl+C to terminate the entire system.
echo ======================================================================
echo.
wsl -d Ubuntu-22.04 bash -c "source /opt/ros/humble/setup.bash && source /home/sarat_9_7/ros2_uav_ws/install/setup.bash && ros2 launch uav_control uav_complete_system.launch.py"
pause
