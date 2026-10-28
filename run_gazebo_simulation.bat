@echo off
title UAV Flight Simulation inside Gazebo 3D Physics & Ground Control Station
cls
echo ======================================================================
echo           UAV FLIGHT SIMULATION INSIDE GAZEBO 3D (ROS 2 HUMBLE)
echo ======================================================================
echo.
echo Initializing:
echo  1. Gazebo 3D Physics Simulator with Quadrotor UAV
echo  2. 6-DOF Flight Controller & Aerodynamic Disturbance Observer (DOB)
echo  3. Rayleigh Fluid Drag & Dynamic Wind Simulation
echo  4. Official Ground Control Station (GCS) Dashboard
echo.
echo Close this window or press Ctrl+C to terminate simulation.
echo ======================================================================
echo.
wsl -d Ubuntu-22.04 bash -c "chmod +x /mnt/d/ROS/run_gazebo_simulation.sh && /mnt/d/ROS/run_gazebo_simulation.sh"
pause
