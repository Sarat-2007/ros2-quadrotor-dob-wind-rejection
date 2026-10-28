@echo off
REM ========================================================================
REM  UAV Autonomous Patrol & Mission Abort Verification Test (WSL2 Launcher)
REM ========================================================================
title UAV Autonomous Patrol & Abort Verification Suite

echo [INFO] Starting UAV Autonomous Patrol & Abort Verification Test in WSL2...
wsl bash -c "cd /mnt/d/ROS && chmod +x run_patrol_verification.sh && ./run_patrol_verification.sh"

echo.
echo [INFO] Test run completed.
pause
