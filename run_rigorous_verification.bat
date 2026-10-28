@echo off
title Rigorous Flight, Thrust & DOB Gust Rejection Automated Verification
echo ======================================================================
echo    UAV RIGOROUS FLIGHT, THRUST & DISTURBANCE OBSERVER TEST
echo ======================================================================
echo 1. Ground Rest Verification (Z = 0.12m Skid Contact)
echo 2. Autonomous Takeoff & Altitude Stabilization (Z = 2.0m)
echo 3. Sudden 8.0 m/s Wind Gust with Active DOB (Zero-Drift Counter-Thrust)
echo 4. Sudden 8.0 m/s Wind Gust with Baseline PID (Drift Comparison)
echo 5. Continuous Manual Pilot Teleoperation (+3.8m uninterrupted flight)
echo 6. Controlled Soft Landing & Automatic Motor Disarm
echo ======================================================================
echo.

wsl bash -c "chmod +x /mnt/d/ROS/run_rigorous_verification.sh && /mnt/d/ROS/run_rigorous_verification.sh"

echo.
pause
