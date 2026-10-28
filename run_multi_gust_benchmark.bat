@echo off
REM ========================================================================
REM  UAV Multi-Position Wind Gust Benchmark: PID vs DOB (WSL2 Launcher)
REM ========================================================================
title UAV Multi-Position Wind Gust Benchmark

echo [INFO] Running Multi-Position Gust Simulation (Baseline PID vs. Active DOB)...
wsl bash -c "cd /mnt/d/ROS && chmod +x run_multi_gust_benchmark.sh && ./run_multi_gust_benchmark.sh"

echo.
echo [INFO] Simulation and Benchmark generation completed!
echo [INFO] Opening generated comparison figure: multi_gust_dob_vs_pid.png
start multi_gust_dob_vs_pid.png
pause
