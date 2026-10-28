@echo off
title UAV Teleoperated Flight Simulation (Dedicated Remote Controller GUI)
cls
echo ======================================================================
echo   UAV TELEOPERATED FLIGHT SIMULATION & DEDICATED PILOT CONTROLLER
echo                           ROS 2 HUMBLE
echo ======================================================================
echo.
echo Launching:
echo  1. 3D Quadrotor Drone Model in RViz2 (starts on ground skids)
echo  2. 6-DOF Flight Dynamics & Real-Time Controller
echo  3. DEDICATED PILOT REMOTE CONTROLLER GUI (Separate Window)
echo.
echo Key Controls on Remote Controller:
echo   [T]     : TAKEOFF to 2.0m Hover
echo   [L]     : LAND safely on ground skids
echo   [H]     : HOVER / Hold position
echo   [W]/[S] : Fly Forward / Backward
echo   [A]/[D] : Strafe Left / Right
echo   [I]/[K] : Climb Up / Descend Down
echo   [J]/[;] : Yaw Rotate Left / Right
echo ======================================================================
echo.
wsl -d Ubuntu-22.04 bash -c "source /opt/ros/humble/setup.bash && source /home/sarat_9_7/ros2_uav_ws/install/setup.bash && ros2 launch uav_control uav_teleop_simulation.launch.py"
pause
