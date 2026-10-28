@echo off
title UAV Keyboard Flight & Teleoperation Console
cls
echo ======================================================================
echo       REAL-TIME UAV KEYBOARD FLIGHT & TELEOPERATION CONSOLE
echo ======================================================================
echo.
echo Make sure "run_full_simulation.bat" is running first!
echo.
echo Controls:
echo   [T]     : TAKEOFF to 2.0m Hover
echo   [L]     : LAND safely on ground skids
echo   [H]     : HOVER / Hold position
echo.
echo   [W]     : Fly Forward (+X)
echo   [S]     : Fly Backward (-X)
echo   [A]     : Strafe Left (+Y)
echo   [D]     : Strafe Right (-Y)
echo   [I]/[E] : Climb Up (+Z)
echo   [K]/[Q] : Descend Down (-Z)
echo   [J]     : Yaw Rotate Left
echo   [;]     : Yaw Rotate Right
echo.
echo   [C]     : Clear Visual Path Trail
echo   [Space] : Stop / Brake to Hover
echo ======================================================================
echo.
wsl -d Ubuntu-22.04 bash -c "source /opt/ros/humble/setup.bash && source /home/sarat_9_7/ros2_uav_ws/install/setup.bash && ros2 run uav_control uav_teleop_keyboard"
pause
