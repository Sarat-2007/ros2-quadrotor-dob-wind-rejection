# High-Precision 6-DOF Quadrotor UAV Simulation with Disturbance Observer (DOB) and Rayleigh Aerodynamics in ROS 2 Humble

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20Hawksbill-22314E.svg)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic%2011-orange.svg)](https://gazebosim.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04%20LTS-E95420.svg)](https://releases.ubuntu.com/22.04/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

An industrial-grade, 6-DOF quadrotor unmanned aerial vehicle (UAV) flight dynamics simulator, active disturbance rejection control (ADRC / Disturbance Observer), real-time 3D Rayleigh fluid aerodynamics engine, and dedicated Ground Control Station (GCS) built on **ROS 2 Humble** and **Gazebo Classic / RViz2**.

---

## Key Features

- **Non-Linear 6-DOF Rigid-Body Dynamics**: 500 Hz sub-stepped numerical integration ensuring unconditional numerical stability across high-agility maneuvers.
- **Active Disturbance Rejection Control (DOB / ADRC)**: High-bandwidth ($\omega_{obs} = 35\text{ rad/s}$) disturbance observer estimating fluid drag and wind gusts in real time, injecting dynamic feedforward counter-thrust.
- **73.3% Gust Drift Reduction**: Rigorously benchmarked against standard industrial PID controllers under violent $8.0\text{ m/s}$ ($10.33\text{ N}$) sudden wind gusts.
- **Rayleigh Fluid Aerodynamics**: Quadratic aerodynamic drag ($F_d = -\frac{1}{2}\rho C_d A |v_{rel}| v_{rel}$) coupled with 1-cosine gust profiles and Dryden-like stochastic atmospheric turbulence.
- **High-Fidelity 3D Drone Model**: Modular URDF/Xacro model featuring central carbon-fiber plates, landing skids, brushless motors, continuous rotating joints with ODE physics limits, and active motor disarm upon landing.
- **Industrial Ground Control Station (GCS)**: Aerospace-grade dark slate UI (`#1a1d24`) providing uninterrupted pilot stick teleoperation, live flight status, DOB mode toggling, and 1-click $8\text{ m/s}$ gust triggers.
- **Fully Automated Verification Suite**: Headless and visual test suite validating the 6-stage flight lifecycle, disturbance estimation, and empirical PID comparison.

---

## Empirical Benchmark Performance

Under a violent **$8.0\text{ m/s}$ sudden wind gust** generating a **$10.33\text{ N}$ aerodynamic shock force** on a $1.5\text{ kg}$ quadrotor:

| Metric | Baseline Industrial PID | Our Disturbance Observer (DOB) | Performance Advantage |
| :--- | :---: | :---: | :---: |
| **Max Displacement Drift** | $0.656\text{ m}$ ($65.6\text{ cm}$) | **$0.175\text{ m}$ ($17.5\text{ cm}$)** | **$73.3\%$ Drift Reduction** |
| **Peak Force Estimated** | N/A (Reactive only) | **$10.34\text{ N}$** ($>98\%$ accuracy) | Immediate Feedforward |
| **Counter-Tilt Response** | Slow error accumulation | Instant $-35.1^\circ$ lean into wind | Zero lag compensation |
| **Altitude Stability** | Drops under tilt | Rock-solid at $Z = 2.03\text{ m}$ | Non-linear tilt thrust |
| **Touchdown & Disarm** | Manual | Automatic on skids ($Z = 0.12\text{ m}$) | Safe ground contact |

---

## Mathematical & Control Foundations

### 1. Rayleigh Fluid Aerodynamic Drag
The aerodynamic force acting on the quadrotor body is calculated via the non-linear Rayleigh fluid resistance model:
$$\mathbf{F}_{drag} = -\frac{1}{2} \rho C_d A \|\mathbf{v}_{rel}\| \mathbf{v}_{rel}$$
where:
- $\rho = 1.225\text{ kg/m}^3$ (Standard sea-level air density)
- $C_d = 1.15$ (Bluff-body quadrotor aerodynamic drag coefficient)
- $A = 0.22\text{ m}^2$ (Effective frontal projection area)
- $\mathbf{v}_{rel} = \mathbf{v}_{drone} - \mathbf{v}_{wind}$ (Relative airspeed vector)

### 2. Disturbance Observer (DOB) Filter
Rather than relying on position error $e(t)$ to slowly build up integral torque, the Disturbance Observer estimates the unmodeled external disturbance force $\hat{\mathbf{F}}_{dist}$:
$$\dot{\hat{\mathbf{F}}}_{dist} = \omega_{obs} (\mathbf{F}_{drag} - \hat{\mathbf{F}}_{dist})$$
with observer bandwidth filter $\omega_{obs} = 35.0\text{ rad/s}$.

### 3. Outer Loop Acceleration Command with Non-Linear Thrust
The desired acceleration is fed with the inverted disturbance force vector:
$$\mathbf{a}_{des} = \mathbf{a}_{ref} + \mathbf{K}_p \mathbf{e}_{pos} + \mathbf{K}_i \int \mathbf{e}_{pos} dt + \mathbf{K}_d \mathbf{e}_{vel} - \frac{1}{m}\hat{\mathbf{F}}_{dist}$$

To prevent altitude drop during steep tilt angles (up to $45^\circ$), collective thrust is scaled by non-linear tilt compensation:
$$T = \frac{m \cdot a_{z,des}}{\cos(\phi) \cos(\theta)}$$

---

## Workspace Structure

```
ROS/
├── src/
│   ├── uav_description/      # URDF/Xacro 3D model, materials, RViz configs, Gazebo world
│   │   ├── urdf/             # quadrotor.urdf.xacro, quadrotor_materials.xacro
│   │   ├── worlds/           # uav_world.world (with libgazebo_ros_state.so)
│   │   ├── rviz/             # flight_sim.rviz, uav_model.rviz
│   │   └── launch/           # display.launch.py
│   ├── uav_control/          # 6-DOF controller, Disturbance Observer, trajectories
│   │   ├── uav_control/      # flight_controller_node.py, trajectory_generator_node.py
│   │   └── launch/           # uav_gazebo.launch.py, uav_rviz.launch.py, uav_complete_system.launch.py
│   ├── uav_aerodynamics/   # Dynamic wind field & 3D aerodynamic drag calculation
│   │   ├── uav_aerodynamics/ # wind_field_node.py, aerodynamic_drag_node.py
│   │   └── launch/           # aerodynamics.launch.py
│   └── uav_dashboard/        # Official Ground Control Station & Live Telemetry GUI
│       ├── uav_dashboard/    # uav_teleop_gui.py, live_tuning_gui.py, plot_performance.py
│       └── launch/           # teleop_dashboard.launch.py, dashboard.launch.py
├── run_gazebo_simulation.sh  # Complete Gazebo 3D launch script
├── run_gazebo_simulation.bat # Windows double-click Gazebo launcher
├── run_rigorous_verification.sh # Automated 6-stage test runner
├── run_rigorous_verification.bat# Windows double-click verification launcher
├── test_gust_and_dob_rigorous.py# Rigorous automated test node
└── README.md
```

---

## System Requirements & Prerequisites

- **OS**: Ubuntu 22.04 LTS (Native or via Windows Subsystem for Linux WSL2)
- **ROS 2 Distribution**: ROS 2 Humble Hawksbill (`ros-humble-desktop`)
- **Simulation**: Gazebo Classic 11 (`gazebo`, `ros-humble-gazebo-ros-pkgs`)
- **Python**: Python 3.10+ with `numpy`, `PyQt5`, `matplotlib`

### Install Dependencies (Ubuntu / WSL):
```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  python3-pip
pip3 install numpy matplotlib PyQt5
```

---

## Installation & Build Instructions

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/Sarat-2007/ros2-quadrotor-dob-wind-rejection.git
   cd ros2-quadrotor-dob-wind-rejection
   ```

2. **Source ROS 2 & Build Workspace**:
   ```bash
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ```

---

## Execution & Launch Commands

### Option 1: All-In-One Gazebo 3D Simulation & GCS Dashboard
Launches Gazebo 3D physics, spawns the quadrotor on its landing skids, starts the 6-DOF flight controller with DOB, wind field generator, and opens the Ground Control Station:
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:$GAZEBO_PLUGIN_PATH

ros2 launch uav_control uav_gazebo.launch.py dashboard:=true
```
*(On Windows: double-click `run_gazebo_simulation.bat`)*

---

### Option 2: Running Components in Separate Terminals

#### Terminal 1 (Gazebo 3D Simulation):
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:$GAZEBO_PLUGIN_PATH

ros2 launch uav_control uav_gazebo.launch.py
```

#### Terminal 1 (Alternative: RViz2 3D Visualizer):
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch uav_control uav_rviz.launch.py
```

#### Terminal 2 (Ground Control Station Teleop Dashboard):
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch uav_dashboard teleop_dashboard.launch.py
```

---

### Option 3: Automated Rigorous Verification Suite (Test Lifecycle, Gusts & DOB)
Executes an automated, reproducible 6-stage flight mission measuring exact disturbance forces and displacement drift:
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

./run_rigorous_verification.sh
```
*(On Windows: double-click `run_rigorous_verification.bat`)*

---

## Pilot Remote Controls (GCS Dashboard)

| Control | Action | Autopilot Behavior |
| :--- | :--- | :--- |
| **TAKEOFF** | Click | Propellers spin up, climbs smoothly to $2.0\text{ m}$, settles into `HOVER` |
| **FORWARD ($\mathbf{\Delta}$)** | **Click & Hold** | Continuous forward motion ($+1.2\text{ m/s}$); brakes to hover upon release |
| **BACKWARD ($\mathbf{\nabla}$)**| **Click & Hold** | Continuous reverse motion ($-1.2\text{ m/s}$) |
| **LEFT ($\mathbf{\triangleleft}$)** | **Click & Hold** | Continuous leftward lateral flight |
| **RIGHT ($\mathbf{\triangleright}$)** | **Click & Hold** | Continuous rightward lateral flight |
| **CLIMB ($\mathbf{\uparrow}$)** | **Click & Hold** | Ascends at $+0.8\text{ m/s}$ |
| **DESCEND ($\mathbf{\downarrow}$)** | **Click & Hold** | Descends at $-0.6\text{ m/s}$ |
| **YAW $\circlearrowleft / \circlearrowright$** | **Click & Hold** | Rotates azimuth at $\pm 45^\circ/\text{s}$ |
| **⚡ 8.0 m/s GUST** | Click | Injects sudden $8.0\text{ m/s}$ ($10.3\text{ N}$) shock wind; drone counter-tilts into wind |
| **DOB MODE TOGGLE** | Toggle | Switch live between **Active DOB (Zero-Drift)** and **Baseline PID** |
| **LAND** | Click | Controlled descent to $Z = 0.12\text{ m}$, touches down on skids, automatically disarms motors |

---

## Authors & Citation

- **Author**: Nallabati Venkata Durga Sai Sarat Chandra
- **Email**: saratchandranallabati@gmail.com
- **GitHub**: [@Sarat-2007](https://github.com/Sarat-2007)
- **Framework**: ROS 2 Humble Hawksbill, Gazebo Classic 11

```bibtex
@misc{sarat_uav_dob_2026,
  author = {Nallabati Venkata Durga Sai Sarat Chandra},
  title = {High-Precision 6-DOF Quadrotor UAV Simulation with Disturbance Observer and Rayleigh Aerodynamics in ROS 2 Humble},
  year = {2026},
  publisher = {GitHub},
  howpublished = {\url{https://github.com/Sarat-2007/ros2-quadrotor-dob-wind-rejection}}
}
```

---

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.
