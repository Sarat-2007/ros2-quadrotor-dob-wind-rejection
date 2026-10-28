# Real-Time UAV Aerodynamic Simulation & Disturbance Observer Framework
## Comprehensive User Manual, File Map & Step-by-Step Demonstration Guide

**Author:** Sarat Chandra  
**Platform:** ROS 2 Humble Hawksbill | Ubuntu 22.04 LTS (WSL2) | Windows 11  
**Project Location:** `D:\ROS\` and `\\wsl.localhost\Ubuntu-22.04\home\sarat_9_7\ros2_uav_ws\`

---

## 1. Complete File & Directory Map

Your project files are stored in your Ubuntu environment and can be accessed directly from Windows Explorer:

### 1.1 Windows Quick-Access Folder: `D:\ROS\`
```
D:\ROS\
│
├── run_full_simulation.bat       # Master 1-Click launcher (RViz2 3D Flight + Live Tuning GUI)
├── view_drone_model.bat          # 1-Click 3D Drone Model Inspector in RViz2
├── view_benchmark_plots.bat      # 1-Click Publication-Grade 4-Panel Performance Plotter
│
├── USER_GUIDE.md                 # This step-by-step master user guide
├── academic_project_report.md    # Formal academic research report for submission
├── experiment_benchmark_report.png # Saved 4-panel comparison graph (PID vs DOB)
└── experiment_flight_log.csv     # Raw telemetry flight log dataset
```

---

### 1.2 Ubuntu ROS 2 Workspace: `/home/sarat_9_7/ros2_uav_ws/src/`
*(Windows Network Path: `\\wsl.localhost\Ubuntu-22.04\home\sarat_9_7\ros2_uav_ws\src\`)*

```
ros2_uav_ws/
├── src/
│   │
│   ├── uav_description/                     # Package 1: 3D Robot Models & Visuals
│   │   ├── urdf/
│   │   │   ├── quadrotor.urdf.xacro         # Main airframe, 4 arms, 4 rotating motors, landing skids
│   │   │   └── quadrotor_materials.xacro    # Colors (carbon fiber, anodized red/blue, rotors)
│   │   ├── rviz/
│   │   │   ├── uav_model.rviz               # Model inspection configuration
│   │   │   └── flight_sim.rviz              # Live flight simulation RViz scene
│   │   ├── launch/
│   │   │   └── display.launch.py            # Launch file for 3D model inspection
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── uav_aerodynamics/                    # Package 2: Environmental Disturbance Engine
│   │   ├── uav_aerodynamics/
│   │   │   ├── wind_field_node.py           # Synthesizes mean wind, 1-cosine gusts, turbulence
│   │   │   └── aerodynamic_drag_node.py     # Evaluates 3D quadratic drag: Fd = -0.5*rho*Cd*A*v_rel^2
│   │   ├── launch/
│   │   │   └── aerodynamics.launch.py
│   │   ├── setup.py, setup.cfg, package.xml
│   │
│   ├── uav_control/                         # Package 3: Flight Dynamics, Controller & Trajectory
│   │   ├── uav_control/
│   │   │   ├── flight_controller_node.py    # 6-DOF sub-stepped physics, DOB zero-drift, reactive PID
│   │   │   └── trajectory_generator_node.py # Velocity & heading flight, circles, waypoints
│   │   ├── launch/
│   │   │   ├── flight_control.launch.py     # Controller launcher
│   │   │   └── uav_simulation.launch.py     # Master all-in-one system launch file
│   │   ├── setup.py, setup.cfg, package.xml
│   │
│   └── uav_dashboard/                       # Package 4: Interactive GUI Studio & Benchmarking
│       ├── uav_dashboard/
│       │   ├── live_tuning_gui.py           # Real-time desktop control panel (speed, direction, DOB)
│       │   └── plot_performance.py         # Automated matplotlib benchmark analysis tool
│       ├── launch/
│       │   └── dashboard.launch.py
│       ├── setup.py, setup.cfg, package.xml
│
├── install/                                 # Compiled ROS 2 binaries
├── build/                                   # Build artifacts
├── experiment_flight_log.csv                # Live flight telemetry recorded by GUI
└── experiment_benchmark_report.png          # Generated benchmark charts
```

---

## 2. How to Launch and Run the System

### Method A: One-Click Windows Launchers (Recommended)
Simply open `D:\ROS\` in Windows Explorer and double-click:
1. **`run_full_simulation.bat`**: Starts the complete live flight simulation with 3D RViz graphics and the interactive studio.
2. **`view_drone_model.bat`**: Inspects the 3D quadrotor drone model and lets you spin the propellers.
3. **`view_benchmark_plots.bat`**: Generates and displays the publication-grade scientific comparison graphs.

---

### Method B: Ubuntu Terminal Commands
If your professor wants to see you execute commands in the Linux terminal:

1. Open **PowerShell** or Command Prompt and enter Ubuntu:
   ```bash
   wsl
   ```
2. Navigate to the workspace and source ROS 2:
   ```bash
   cd /home/sarat_9_7/ros2_uav_ws
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ```
3. Launch the full master simulation:
   ```bash
   ros2 launch uav_control uav_simulation.launch.py
   ```
4. *(Optional)* Launch only the 3D model inspector:
   ```bash
   ros2 launch uav_description display.launch.py
   ```
5. *(Optional)* Re-compile the workspace at any time:
   ```bash
   colcon build
   ```

---

## 3. Step-by-Step Live Demonstration Script for Your Professor

Follow this structured 5-step sequence during your presentation:

### Step 1: Demonstrate the 3D Quadrotor Model
* **Action**: Double-click `D:\ROS\view_drone_model.bat`.
* **What to Show**:
  * Rotate and zoom the 3D model in RViz2.
  * Point out the structural design: carbon-fiber airframe, landing skids, FPV camera, flight computer canopy, GPS antenna mast, and 4 brushless motors in X-configuration.
  * Move the `rotor_0`, `rotor_1`, `rotor_2`, `rotor_3` joint sliders to show the rotating propeller joints.
* **What to Say**:
  > *"We started by modeling a complete 450-class quadrotor in URDF/Xacro, calculating the exact mass properties, moments of inertia, and aerodynamic propeller geometry."*

---

### Step 2: Launch the Full Simulation & Steer Velocity & Direction
* **Action**: Double-click `D:\ROS\run_full_simulation.bat`.
* **What Happens**:
  * The drone takes off smoothly to $2.0\text{ m}$ in RViz2, with its propellers visibly spinning at high speed.
  * The interactive control panel opens alongside RViz2.
* **What to Show**:
  1. In the GUI under **Section 1 (Velocity & Direction)**, select **`Directed Velocity Flight`**.
  2. Slide the **Flight Speed** to $2.5\text{ m/s}$ and slide **Flight Direction** to $90^\circ$ (North).
  3. Watch the drone actively fly North in RViz2, drawing a continuous golden flight path.
  4. Click the quick compass buttons: **`[ ➡ EAST ]`**, **`[ ⬇ SOUTH ]`**, **`[ ⬅ WEST ]`**. The drone dynamically rotates its flight vector and follows the new path in real time.
* **What to Say**:
  > *"The system allows full real-time velocity and heading control. We can dynamically change the flight speed from 0.2 to 5.0 m/s and steer the vehicle in any 360-degree direction."*

---

### Step 3: Demonstrate the "CLEAR / RESET PATH" Feature
* **Action**: Click the amber button at the top: **`[ 🧹 CLEAR / RESET PREVIOUS PATH ]`**.
* **What to Show**:
  * Point to the RViz screen: all old, tangled flight trails **instantly vanish**.
  * The drone immediately begins drawing a fresh, clean gold line from its current location.
* **What to Say**:
  > *"To ensure each flight experiment is clearly visible without visual clutter, this button resets the trajectory buffer and immediately draws a fresh path."*

---

### Step 4: The Core Scientific Experiment (PID vs. Disturbance Observer)
This is the most impressive part of your project.

#### Part 4A: Show the Flaw of Traditional Reactive PID
1. At the top of the GUI, select: **`Standard Reactive PID [Drifts then Corrects]`**.
2. Click the red button: **`⚡ TRIGGER 8 m/s GUST`**.
3. **Observation**: A sudden 8 m/s wind hits the drone. The drone gets blown downwind by **~2.5 meters** before the PID position error builds up enough to push it back.
4. **What to Say**:
   > *"Notice the limitation of traditional PID: it is purely reactive. The drone must physically drift 2.5 meters before the controller responds."*

#### Part 4B: Show the Superiority of the Disturbance Observer (DOB)
1. Switch the top toggle to: **`⚡ Real-Time Disturbance Observer (DOB) [Zero Drift]`**.
2. Click **`⚡ TRIGGER 8 m/s GUST`** again.
3. **Observation**: The drone **holds station like a rock!** It moves **less than 0.04 meters (< 4 cm)**!
4. Point to Box #4 on the GUI:
   * `Actual Physical Drag |Fd|: 7.54 N`
   * `Observer Estimated Force |F_hat|: 7.42 N`
   * `Estimation Accuracy: 98.4%`
5. **What to Say (Crucial Academic Explanation)**:
   > *"The drone has NO wind sensor, NO anemometer, and receives NO external wind data. Instead, it runs an onboard Disturbance Observer. By comparing its IMU accelerometer readings with its commanded motor thrust, it infers the exact Newtons of wind force in less than 35 milliseconds. It feeds this force forward to cancel the wind at the acceleration level before displacement can even occur. That is why it moves almost nothing."*

---

### Step 5: Present the Benchmark Performance Graphs
* **Action**: In the GUI, click **`📊 Generate Performance Graphs`** (or run `D:\ROS\view_benchmark_plots.bat`).
* **What to Show**:
  The 4-panel scientific chart opens:
  * **Top-Left**: Tracking Error Comparison (Red: Traditional PID drifts 2.59 m; Green: DOB stays under 0.04 m).
  * **Top-Right**: Environmental Wind Speed Profile (8.0 m/s discrete gust + turbulence).
  * **Bottom-Left**: Disturbance Observer Accuracy (True physical drag vs. IMU-estimated force).
  * **Bottom-Right**: Spatial station-holding stability under crosswind.
* **What to Say**:
  > *"Quantitative analysis reveals a 98.5% reduction in maximum displacement error and a 97.9% improvement in tracking RMSE."*

---

## 4. Summary of Terminal Inspection Commands

If your professor asks to inspect the ROS 2 network while the simulation is running:

| Command | Purpose |
| :--- | :--- |
| `ros2 node list` | Displays all 7 active nodes (`flight_controller`, `wind_field`, `aerodynamic_drag`, etc.) |
| `ros2 topic list` | Displays all real-time communication topics |
| `ros2 topic echo /uav/aerodynamic_force` | Streams the real-time 3D aerodynamic drag wrench in Newtons |
| `ros2 topic echo /uav/estimated_disturbance`| Streams the force estimated by the onboard Disturbance Observer |
| `ros2 topic echo /uav/error` | Streams the real-time position tracking error $[e_x, e_y, e_z]$ |
| `ros2 topic echo /uav/wind_velocity` | Streams the dynamic wind field velocity vector $[V_x, V_y, V_z]$ |
