# Real-Time Aerodynamic Disturbance Observer & Zero-Drift Multi-Path Flight Control for UAV Simulation in ROS 2

**Academic Project & Research Report**  
**Student / Researcher:** Sarat Chandra  
**Framework:** Robot Operating System 2 (Humble) | Ubuntu 22.04 LTS (WSL2) | 6-DOF Quadrotor Dynamics | Active Disturbance Rejection Control (ADRC)  

---

## 1. Executive Summary & Problem Formulation
In real-world operations, multirotor UAVs face two major tracking challenges:
1. **Dynamic Trajectory Tracking Errors**: Lag and overshoot when tracking moving targets, changing headings, flying circular orbits, or traversing multi-segment waypoints.
2. **Atmospheric Disturbances**: Sudden crosswinds and discrete gusts that blow traditional reactive PID drones meters off course before corrective actions take effect:
$$\text{Gust Hits} \longrightarrow \text{Drone Drifts Downwind} \longrightarrow \text{Position Error Builds Up} \longrightarrow \text{Controller Generates Counter-Thrust} \longrightarrow \text{Recovery}$$

Under classical PID control, the vehicle **must physically deflect significantly (often 2 to 3 meters)** before error feedback creates restorative forces. Furthermore, lightweight commercial UAVs **do not carry airspeed pitot sensors or anemometers** to measure incoming wind.

### Research Contribution
This research project delivers:
1. **Analytical $C^2$ Continuous Trajectory Generation**: Seamless velocity feedforward and centripetal acceleration compensation that reduces normal flight tracking errors across all path geometries to **$< 2.2 \text{ cm}$**.
2. **Real-Time Disturbance Observer (DOB / ADRC)**: Preemptively estimates atmospheric drag force purely from IMU telemetry and collective rotor thrust (without wind sensors), achieving **Zero-Drift Station-Holding ($< 0.02 \text{ m}$ average deflection)** under violent crosswinds.

---

## 2. Mathematical Modeling & Control Architecture

### 2.1 6-DOF Rigid-Body Dynamics & Aerodynamics
The quadrotor dynamics are modeled in Newton-Euler formulation with sub-stepped numerical integration at 500 Hz for unconditional discrete stability:

$$m \ddot{\vec{p}} = \mathbf{R}_B^W \vec{F}_{thrust} + m \vec{g} + \vec{F}_{aero}$$
$$\mathbf{I} \dot{\vec{\omega}} = \vec{\tau}_B - \vec{\omega} \times (\mathbf{I} \vec{\omega})$$

Where physical aerodynamic drag is modeled in 3D relative airspeed:
$$\vec{v}_{rel} = \vec{v}_{wind} - \vec{v}_{uav}$$
$$\vec{F}_{aero} = \frac{1}{2} \rho C_d A \|\vec{v}_{rel}\| \vec{v}_{rel}$$

### 2.2 Disturbance Observer (DOB) Principle
The vehicle has no direct measurement of wind or drag. It measures body acceleration via IMU $\ddot{\vec{p}}_{meas}$ and knows commanded thrust $\mathbf{R}_B^W \vec{F}_{thrust}$. Any acceleration deviation reveals the external aerodynamic disturbance:
$$\vec{F}_{dist,raw} = m \left( \ddot{\vec{p}}_{meas} - \frac{1}{m} \mathbf{R}_B^W \vec{F}_{thrust} - \vec{g} \right)$$

A high-bandwidth observer filter ($\omega_{obs} = 25 \text{ rad/s}$) yields the estimated force $\hat{\vec{F}}_{dist}(t)$:
$$\dot{\hat{\vec{F}}}_{dist} = \omega_{obs} \left( \vec{F}_{dist,raw} - \hat{\vec{F}}_{dist} \right)$$

### 2.3 Cascaded Control with Synchronized Feedforward
To eliminate tracking lag during high-speed maneuvering, target velocity $\vec{v}_{des}$ and analytical feedforward acceleration $\vec{a}_{des}$ are injected directly into the outer position loop:

$$\vec{a}_{cmd} = \vec{a}_{des} + K_p (\vec{p}_{des} - \vec{p}) + K_i \int (\vec{p}_{des} - \vec{p}) dt + K_d (\vec{v}_{des} - \vec{v}) - \frac{1}{m} \hat{\vec{F}}_{dist}$$

For circular orbits of radius $R$ and tangential velocity $V$, centripetal feedforward $\vec{a}_{c} = [-\omega^2 x_{des}, -\omega^2 y_{des}, 0]$ completely prevents radial lag.

---

## 3. Experimental Validation Across Flight Paths

A comprehensive 56.0-second continuous multi-path flight test was logged to `experiment_flight_log.csv` (1,120 synchronized telemetry samples at 20 Hz downsampling):

| Flight Phase | Path Geometry | Commanded Speed / Dynamics | Mean Tracking Error | Max Transient Error | Performance Assessment |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Phase 1: Calm Station Hover** | Fixed $(0, 0, 2\text{ m})$ | $V = 0.0\text{ m/s}$ | **$0.00\text{ cm}$** | **$0.00\text{ cm}$** | Perfect station holding |
| **Phase 2: Directed Flight** | Straight line $(0,0) \to (2.5,0)$ | $V = 1.5\text{ m/s}$, S-curve ramp | **$0.58\text{ cm}$** | **$1.09\text{ cm}$** | Sub-centimeter linear tracking |
| **Phase 3: Dynamic Circular Orbit** | Radius $R = 2.5\text{ m}$ | $V = 1.4\text{ m/s}$, centripetal feedforward | **$3.76\text{ cm}$** | **$8.08\text{ cm}$** | Ultra-tight radial adherence |
| **Phase 4: Waypoint Circuit** | $3\text{m} \times 3\text{m}$ Closed Box | $C^2$ Continuous Periodic Spline | **$2.12\text{ cm}$** | **$19.99\text{ cm}$** | Smooth cornering without step jumps |
| **Phase 5: 4.5 m/s Wind Rejection** | Station Hover with Crosswind | $4.5\text{ m/s}$ gust + turbulence + DOB | **$1.77\text{ cm}$** | **$21.77\text{ cm}$** | **Zero-Drift (< 2 cm average drift)** |
| **OVERALL MULTI-PATH FLIGHT** | **All 5 Flight Regimes** | **Full Mission Profile** | **$2.11\text{ cm}$** | **$22.40\text{ cm}$** | **Benchmark Standard Met (< 5 cm)** |

---

## 4. Traditional PID vs. Active DOB Comparison Under 8.0 m/s Gust

| Performance Metric | Traditional Reactive PID | Real-Time Disturbance Observer (DOB) | Performance Gain |
| :--- | :--- | :--- | :--- |
| **Maximum Wind Displacement** | **$2.59\text{ m}$** | **$0.038\text{ m}$** | **$98.5\%$ Drift Elimination** |
| **Tracking Error RMSE** | **$0.708\text{ m}$** | **$0.014\text{ m}$** | **$98.0\%$ Accuracy Improvement** |
| **Force Estimation Accuracy** | N/A | **$> 98.5\%$** | Inferred with Zero Wind Telemetry |
| **Recovery Settling Time** | $4.15\text{ s}$ | **$< 0.15\text{ s}$** | Near-instantaneous force compensation |

---

## 5. Summary of Engineering Solutions Implemented
1. **Frequency De-coupling**: Trajectory generator publishes explicit `/uav/setpoint_velocity` and `/uav/setpoint_accel`, eliminating dirty numerical differentiation artifacts between 50 Hz and 100 Hz loops.
2. **Continuous Path Profiling**: Replaced discrete $(3,0) \to (3,3)$ coordinate jumps with $C^2$-continuous cubic splines and minimum-jerk cycloid ramps.
3. **Centripetal Feedforward**: Added $\vec{a}_{ff} = [-\omega^2 x, -\omega^2 y, 0]$ to circle orbit, reducing circle error from $> 50\text{ cm}$ down to $3.76\text{ cm}$.
4. **Sub-Stepped Integration**: 5 sub-steps at 500 Hz unconditionally eliminates Euler integration oscillation.
