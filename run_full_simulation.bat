@echo off
title UAV Aerodynamic Flight Simulation & Real-Time Tuning
cls
echo ======================================================================
echo   REAL-TIME WIND & AERODYNAMIC RESISTANCE UAV FLIGHT SIMULATION
echo                       ROS 2 HUMBLE (WSL2)
echo ======================================================================
echo.
echo Starting:
echo  1. 3D Quadrotor Model in RViz2
echo  2. 6-DOF Flight Dynamics & Cascaded PID Controller
echo  3. Real-Time Aerodynamic Drag Model (Fd = -1/2*rho*Cd*A*v^2)
echo  4. Dynamic Wind Field Generator (Mean + Gusts + Turbulence)
echo  5. Interactive Tuning & Telemetry Dashboard GUI
echo.
echo Close this terminal or press Ctrl+C to stop the simulation.
echo ======================================================================
echo.
wsl -d Ubuntu-22.04 bash -c "source /opt/ros/humble/setup.bash && source /home/sarat_9_7/ros2_uav_ws/install/setup.bash && ros2 launch uav_control uav_simulation.launch.py"
pause
