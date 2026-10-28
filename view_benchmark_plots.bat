@echo off
title UAV Flight Performance Benchmark Plots
cls
echo ======================================================================
echo   Generating Publication-Ready Benchmark Performance Analysis Plots
echo ======================================================================
echo.
wsl -d Ubuntu-22.04 bash -c "source /opt/ros/humble/setup.bash && source /home/sarat_9_7/ros2_uav_ws/install/setup.bash && python3 /home/sarat_9_7/ros2_uav_ws/src/uav_dashboard/uav_dashboard/plot_performance.py /home/sarat_9_7/ros2_uav_ws/experiment_flight_log.csv && cp /home/sarat_9_7/ros2_uav_ws/experiment_benchmark_report.png /mnt/d/ROS/experiment_benchmark_report.png"
if exist experiment_benchmark_report.png (
    echo Opening generated benchmark report in Windows...
    start experiment_benchmark_report.png
)
pause

