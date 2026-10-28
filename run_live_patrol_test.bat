@echo off
REM ========================================================================
REM  UAV 3D Obstacle Course Live Patrol & Collision Verification (WSL2)
REM ========================================================================
title UAV 3D Live Patrol & Collision Verification Test

echo [INFO] Running full 11-waypoint 3D obstacle avoidance and collision test in WSL2...
wsl bash -c "cd /mnt/d/ROS && chmod +x run_live_patrol_test.sh && ./run_live_patrol_test.sh"

echo.
echo [INFO] Live patrol test finished.
pause
