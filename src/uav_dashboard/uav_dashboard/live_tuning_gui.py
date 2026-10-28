#!/usr/bin/env python3
"""
Interactive Real-Time Flight, Velocity, Direction & Teleoperation Studio
Features:
- Complete Flight Lifecycle: Takeoff [T], Hover [H], Keyboard Teleop, Landing [L]
- Interactive Graphical D-Pad + Direct Keyboard Binding (W/A/S/D, Arrow keys, I/K, J/;)
- Dynamic Flight Velocity & Direction Sliders
- CLEAR / RESET PATH Button & RETURN TO HOME Button
- Dynamic Wind (Speed, Direction, Turbulence, Instant 8 m/s Gust Trigger)
- Aerodynamic Parameters (Cd, Area A)
- Disturbance Observer (DOB / ADRC) ZERO-DRIFT Mode Toggle
- Real Drag vs Estimated Drag Observer Accuracy Readout
- Telemetry Display & CSV Data Logger
"""

import sys
import os
import math
import time
import threading
import csv
import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped, Vector3, WrenchStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int32, Bool, Empty, String

class RosGuiBridge(Node):
    def __init__(self):
        super().__init__('uav_tuning_gui_node')

        # Publishers: Flight Lifecycle & Teleoperation
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.takeoff_pub = self.create_publisher(Empty, '/uav/cmd_takeoff', 10)
        self.land_pub = self.create_publisher(Empty, '/uav/cmd_land', 10)
        self.hover_pub = self.create_publisher(Empty, '/uav/cmd_hover', 10)
        self.clear_path_pub = self.create_publisher(Empty, '/uav/clear_path', 10)
        self.reset_home_pub = self.create_publisher(Empty, '/uav/reset_to_home', 10)

        # Publishers: Tuning & Aerodynamics
        self.wind_pub = self.create_publisher(Vector3, '/uav/set_wind', 10)
        self.gust_pub = self.create_publisher(Float32, '/uav/trigger_gust', 10)
        self.aero_pub = self.create_publisher(Vector3, '/uav/set_aero_params', 10)
        self.traj_pub = self.create_publisher(Int32, '/uav/set_trajectory_mode', 10)
        self.speed_pub = self.create_publisher(Float32, '/uav/set_flight_speed', 10)
        self.heading_pub = self.create_publisher(Float32, '/uav/set_flight_heading', 10)
        self.pid_pub = self.create_publisher(Vector3, '/uav/set_pid_gains', 10)
        self.dob_pub = self.create_publisher(Bool, '/uav/set_dob_mode', 10)

        # Telemetry State (Thread-safe)
        self.lock = threading.Lock()
        self.pos = [0.0, 0.0, 0.12]
        self.vel = [0.0, 0.0, 0.0]
        self.wind = [0.0, 0.0, 0.0]
        self.drag = [0.0, 0.0, 0.0]
        self.est_drag = [0.0, 0.0, 0.0]
        self.error = [0.0, 0.0, 0.0]
        self.flight_status = "LANDED"

        # Subscribers
        self.create_subscription(Odometry, '/uav/odom', self.odom_cb, 10)
        self.create_subscription(Vector3Stamped, '/uav/wind_velocity', self.wind_cb, 10)
        self.create_subscription(WrenchStamped, '/uav/aerodynamic_force', self.drag_cb, 10)
        self.create_subscription(WrenchStamped, '/uav/estimated_disturbance', self.est_drag_cb, 10)
        self.create_subscription(Vector3Stamped, '/uav/error', self.error_cb, 10)
        self.create_subscription(String, '/uav/flight_status', self.status_cb, 10)

    def odom_cb(self, msg: Odometry):
        with self.lock:
            self.pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
            self.vel = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]

    def wind_cb(self, msg: Vector3Stamped):
        with self.lock:
            self.wind = [msg.vector.x, msg.vector.y, msg.vector.z]

    def drag_cb(self, msg: WrenchStamped):
        with self.lock:
            self.drag = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]

    def est_drag_cb(self, msg: WrenchStamped):
        with self.lock:
            self.est_drag = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]

    def error_cb(self, msg: Vector3Stamped):
        with self.lock:
            self.error = [msg.vector.x, msg.vector.y, msg.vector.z]

    def status_cb(self, msg: String):
        with self.lock:
            self.flight_status = msg.data

class LiveTuningApp:
    def __init__(self, root, ros_node: RosGuiBridge):
        self.root = root
        self.ros_node = ros_node
        self.root.title("Real-Time UAV Flight & Disturbance Studio")
        self.root.geometry("960x900")
        self.root.configure(bg='#1e1e24')

        self.recording = False
        self.log_data = []
        self.record_start_time = 0.0

        # Active keys pressed
        self.active_keys = set()
        self.teleop_speed = 1.2
        self.teleop_climb_speed = 0.8
        self.teleop_yaw_speed = 0.8

        self.setup_ui()
        self.bind_keyboard()
        self.update_telemetry_loop()
        self.teleop_publisher_loop()

    def bind_keyboard(self):
        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.bind('<KeyRelease>', self.on_key_release)

    def setup_ui(self):
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TFrame', background='#1e1e24')
        style.configure('TLabelframe', background='#1e1e24', foreground='#58a6ff')
        style.configure('TLabelframe.Label', background='#1e1e24', foreground='#58a6ff', font=('Helvetica', 10, 'bold'))
        style.configure('TLabel', background='#1e1e24', foreground='#e6edf3', font=('Helvetica', 9))
        style.configure('Value.TLabel', background='#1e1e24', foreground='#7ee787', font=('Helvetica', 9, 'bold'))

        main_frame = ttk.Frame(self.root, padding="8")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Title
        title_lbl = tk.Label(main_frame, text="Real-Time UAV Flight Control, Teleoperation & Aerodynamics Studio", 
                             font=('Helvetica', 12, 'bold'), bg='#1e1e24', fg='#58a6ff')
        title_lbl.pack(pady=(0, 2))

        # ======================================================================
        # PROMINENT SECTION 0: FLIGHT LIFECYCLE & TELEOPERATION BAR
        # ======================================================================
        flight_bar = tk.Frame(main_frame, bg='#161b22', bd=1, relief='solid', padx=8, pady=6)
        flight_bar.pack(fill=tk.X, pady=(0, 6))

        # Live Flight Status Banner
        self.lbl_flight_state = tk.Label(flight_bar, text="FLIGHT STATUS: ● ON GROUND (LANDED)", 
                                         font=('Helvetica', 11, 'bold'), bg='#161b22', fg='#e3b341')
        self.lbl_flight_state.pack(side=tk.LEFT, padx=(0, 15))

        # Big 3 Lifecycle Action Buttons
        self.btn_takeoff = tk.Button(flight_bar, text="🚀 TAKEOFF [Key: T]", bg='#238636', fg='white',
                                     font=('Helvetica', 10, 'bold'), relief='flat', padx=12, pady=4, command=self.on_takeoff)
        self.btn_takeoff.pack(side=tk.LEFT, padx=3)

        self.btn_hover = tk.Button(flight_bar, text="🎯 HOVER [Key: H]", bg='#1f6feb', fg='white',
                                   font=('Helvetica', 10, 'bold'), relief='flat', padx=12, pady=4, command=self.on_hover)
        self.btn_hover.pack(side=tk.LEFT, padx=3)

        self.btn_land = tk.Button(flight_bar, text="🛬 LAND [Key: L]", bg='#d29922', fg='#0d1117',
                                  font=('Helvetica', 10, 'bold'), relief='flat', padx=12, pady=4, command=self.on_land)
        self.btn_land.pack(side=tk.LEFT, padx=3)

        self.btn_clear_path = tk.Button(flight_bar, text="🧹 Clear Trail [C]", bg='#30363d', fg='#c9d1d9',
                                        font=('Helvetica', 9, 'bold'), relief='flat', padx=8, pady=4, command=self.on_clear_path)
        self.btn_clear_path.pack(side=tk.RIGHT, padx=2)

        # Strategy Selection Banner
        mode_banner = tk.Frame(main_frame, bg='#161b22', bd=1, relief='solid', padx=8, pady=4)
        mode_banner.pack(fill=tk.X, pady=(0, 6))

        tk.Label(mode_banner, text="CONTROL MODE:", font=('Helvetica', 9, 'bold'), bg='#161b22', fg='#c9d1d9').pack(side=tk.LEFT, padx=(0, 8))
        self.ctrl_mode_var = tk.BooleanVar(value=True)
        r_dob = tk.Radiobutton(mode_banner, text="⚡ Active Disturbance Observer (DOB) [Zero Drift]", 
                               variable=self.ctrl_mode_var, value=True, command=self.on_ctrl_mode_change,
                               bg='#161b22', fg='#3fb950', selectcolor='#0d1117', font=('Helvetica', 9, 'bold'), activebackground='#161b22', activeforeground='#3fb950')
        r_dob.pack(side=tk.LEFT, padx=6)

        r_pid = tk.Radiobutton(mode_banner, text="Traditional Reactive PID [Drifts in Wind]", 
                               variable=self.ctrl_mode_var, value=False, command=self.on_ctrl_mode_change,
                               bg='#161b22', fg='#e6edf3', selectcolor='#0d1117', font=('Helvetica', 9), activebackground='#161b22', activeforeground='#e6edf3')
        r_pid.pack(side=tk.LEFT, padx=6)

        # Two Columns Layout
        cols = ttk.Frame(main_frame)
        cols.pack(fill=tk.BOTH, expand=True)
        col_left = ttk.Frame(cols)
        col_left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 4))
        col_right = ttk.Frame(cols)
        col_right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(4, 0))

        # ======================================================================
        # SECTION 1: KEYBOARD TELEOPERATION & MANUAL FLIGHT D-PAD
        # ======================================================================
        teleop_frame = ttk.LabelFrame(col_left, text=" 1. Manual Flight Controls & Keyboard Teleoperation ", padding="8")
        teleop_frame.pack(fill=tk.X, pady=3)

        dpad_container = tk.Frame(teleop_frame, bg='#1e1e24')
        dpad_container.pack(pady=3)

        # Row 1: Forward & Climb
        r1 = tk.Frame(dpad_container, bg='#1e1e24')
        r1.pack()
        tk.Button(r1, text="⬆ Forward [W / Up]", width=16, bg='#30363d', fg='#58a6ff', font=('Helvetica', 9, 'bold'),
                  command=lambda: self.pulse_teleop(vx=self.teleop_speed)).pack(side=tk.LEFT, padx=2, pady=1)
        tk.Button(r1, text="🔼 Climb [I / E]", width=14, bg='#30363d', fg='#7ee787', font=('Helvetica', 9, 'bold'),
                  command=lambda: self.pulse_teleop(vz=self.teleop_climb_speed)).pack(side=tk.LEFT, padx=2, pady=1)

        # Row 2: Left, Stop, Right
        r2 = tk.Frame(dpad_container, bg='#1e1e24')
        r2.pack()
        tk.Button(r2, text="⬅ Left [A / Left]", width=14, bg='#30363d', fg='#58a6ff', font=('Helvetica', 9, 'bold'),
                  command=lambda: self.pulse_teleop(vy=self.teleop_speed)).pack(side=tk.LEFT, padx=2, pady=1)
        tk.Button(r2, text="⏸ Hover [H / Spc]", width=14, bg='#1f6feb', fg='white', font=('Helvetica', 9, 'bold'),
                  command=self.on_hover).pack(side=tk.LEFT, padx=2, pady=1)
        tk.Button(r2, text="➡ Right [D / Right]", width=14, bg='#30363d', fg='#58a6ff', font=('Helvetica', 9, 'bold'),
                  command=lambda: self.pulse_teleop(vy=-self.teleop_speed)).pack(side=tk.LEFT, padx=2, pady=1)

        # Row 3: Backward & Descend
        r3 = tk.Frame(dpad_container, bg='#1e1e24')
        r3.pack()
        tk.Button(r3, text="⬇ Backward [S / Down]", width=16, bg='#30363d', fg='#58a6ff', font=('Helvetica', 9, 'bold'),
                  command=lambda: self.pulse_teleop(vx=-self.teleop_speed)).pack(side=tk.LEFT, padx=2, pady=1)
        tk.Button(r3, text="🔽 Descend [K / Q]", width=14, bg='#30363d', fg='#f85149', font=('Helvetica', 9, 'bold'),
                  command=lambda: self.pulse_teleop(vz=-self.teleop_climb_speed)).pack(side=tk.LEFT, padx=2, pady=1)

        # Row 4: Yaw Left & Yaw Right
        r4 = tk.Frame(dpad_container, bg='#1e1e24')
        r4.pack(pady=(2, 0))
        tk.Button(r4, text="↺ Rotate Left [J]", width=15, bg='#21262d', fg='#d2a8ff', font=('Helvetica', 8, 'bold'),
                  command=lambda: self.pulse_teleop(vyaw=self.teleop_yaw_speed)).pack(side=tk.LEFT, padx=2)
        tk.Button(r4, text="↻ Rotate Right [;]", width=15, bg='#21262d', fg='#d2a8ff', font=('Helvetica', 8, 'bold'),
                  command=lambda: self.pulse_teleop(vyaw=-self.teleop_yaw_speed)).pack(side=tk.LEFT, padx=2)

        # Teleop Speed Slider
        spd_box = ttk.Frame(teleop_frame)
        spd_box.pack(fill=tk.X, pady=(4, 0))
        ttk.Label(spd_box, text="Teleop Speed:").pack(side=tk.LEFT)
        self.lbl_tele_spd = ttk.Label(spd_box, text="1.20 m/s", style='Value.TLabel')
        self.lbl_tele_spd.pack(side=tk.RIGHT)
        self.scale_tele_spd = ttk.Scale(teleop_frame, from_=0.3, to=3.5, value=1.2, command=self.on_tele_spd_change)
        self.scale_tele_spd.pack(fill=tk.X, pady=1)

        # ======================================================================
        # SECTION 2: AUTONOMOUS TRAJECTORY MODES
        # ======================================================================
        traj_frame = ttk.LabelFrame(col_left, text=" 2. Autonomous Trajectory Modes ", padding="8")
        traj_frame.pack(fill=tk.X, pady=3)

        self.traj_var = tk.IntVar(value=0)
        modes = [
            ("Stationary Hover (0, 0, 2m)", 0),
            ("Directed Velocity Flight (Straight Line)", 1),
            ("Dynamic Circular Orbit (R=2.5m)", 2),
            ("Smooth Spline Waypoint Circuit (3x3m)", 3)
        ]
        for text, val in modes:
            r = tk.Radiobutton(traj_frame, text=text, variable=self.traj_var, value=val, command=self.on_traj_change,
                               bg='#1e1e24', fg='#e6edf3', selectcolor='#0d1117', font=('Helvetica', 9),
                               activebackground='#1e1e24', activeforeground='#58a6ff')
            r.pack(anchor=tk.W, pady=1)

        # Cruising Speed Slider
        s_box = ttk.Frame(traj_frame)
        s_box.pack(fill=tk.X, pady=(3, 0))
        ttk.Label(s_box, text="Autonomous Cruising Speed:").pack(side=tk.LEFT)
        self.lbl_flight_speed = ttk.Label(s_box, text="1.50 m/s", style='Value.TLabel')
        self.lbl_flight_speed.pack(side=tk.RIGHT)
        self.scale_flight_speed = ttk.Scale(traj_frame, from_=0.2, to=4.0, value=1.5, command=self.on_flight_speed_change)
        self.scale_flight_speed.pack(fill=tk.X, pady=1)

        # Heading Slider
        h_box = ttk.Frame(traj_frame)
        h_box.pack(fill=tk.X, pady=(2, 0))
        ttk.Label(h_box, text="Directed Flight Heading:").pack(side=tk.LEFT)
        self.lbl_flight_dir = ttk.Label(h_box, text="0 deg (East)", style='Value.TLabel')
        self.lbl_flight_dir.pack(side=tk.RIGHT)
        self.scale_flight_dir = ttk.Scale(traj_frame, from_=0.0, to=360.0, value=0.0, command=self.on_flight_heading_change)
        self.scale_flight_dir.pack(fill=tk.X, pady=1)

        # ======================================================================
        # SECTION 3: ENVIRONMENTAL WIND SIMULATION
        # ======================================================================
        wind_frame = ttk.LabelFrame(col_right, text=" 3. Atmospheric Wind Field Disturbance ", padding="8")
        wind_frame.pack(fill=tk.X, pady=3)

        w1 = ttk.Frame(wind_frame)
        w1.pack(fill=tk.X, pady=1)
        ttk.Label(w1, text="Mean Wind Speed:").pack(side=tk.LEFT)
        self.lbl_wind_speed = ttk.Label(w1, text="0.0 m/s", style='Value.TLabel')
        self.lbl_wind_speed.pack(side=tk.RIGHT)
        self.scale_wind_speed = ttk.Scale(wind_frame, from_=0.0, to=15.0, value=0.0, command=self.on_wind_change)
        self.scale_wind_speed.pack(fill=tk.X, pady=1)

        w2 = ttk.Frame(wind_frame)
        w2.pack(fill=tk.X, pady=1)
        ttk.Label(w2, text="Wind Direction:").pack(side=tk.LEFT)
        self.lbl_wind_dir = ttk.Label(w2, text="0 deg (East / +X)", style='Value.TLabel')
        self.lbl_wind_dir.pack(side=tk.RIGHT)
        self.scale_wind_dir = ttk.Scale(wind_frame, from_=0.0, to=360.0, value=0.0, command=self.on_wind_change)
        self.scale_wind_dir.pack(fill=tk.X, pady=1)

        w3 = ttk.Frame(wind_frame)
        w3.pack(fill=tk.X, pady=1)
        ttk.Label(w3, text="Turbulence Intensity:").pack(side=tk.LEFT)
        self.lbl_turb = ttk.Label(w3, text="0%", style='Value.TLabel')
        self.lbl_turb.pack(side=tk.RIGHT)
        self.scale_turb = ttk.Scale(wind_frame, from_=0.0, to=1.0, value=0.0, command=self.on_wind_change)
        self.scale_turb.pack(fill=tk.X, pady=1)

        btn_box = ttk.Frame(wind_frame)
        btn_box.pack(fill=tk.X, pady=3)
        tk.Button(btn_box, text="⚡ TRIGGER 8 m/s GUST", bg='#da3633', fg='white',
                  font=('Helvetica', 8, 'bold'), relief='flat', command=self.trigger_gust).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        tk.Button(btn_box, text="🍃 RESET CALM AIR", bg='#30363d', fg='#c9d1d9',
                  font=('Helvetica', 8, 'bold'), relief='flat', command=self.reset_calm).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        # ======================================================================
        # SECTION 4: REAL-TIME TELEMETRY & OBSERVER READOUT
        # ======================================================================
        tele_frame = ttk.LabelFrame(col_right, text=" 4. Real-Time Telemetry & Disturbance Observer ", padding="8")
        tele_frame.pack(fill=tk.X, pady=3)

        self.lbl_tele_pos = ttk.Label(tele_frame, text="Position: [0.00, 0.00, 0.12] m")
        self.lbl_tele_pos.pack(anchor=tk.W, pady=1)

        self.lbl_tele_vel = ttk.Label(tele_frame, text="Ground Speed: 0.00 m/s")
        self.lbl_tele_vel.pack(anchor=tk.W, pady=1)

        self.lbl_tele_wind = ttk.Label(tele_frame, text="Wind Velocity: 0.00 m/s")
        self.lbl_tele_wind.pack(anchor=tk.W, pady=1)

        self.lbl_real_drag = ttk.Label(tele_frame, text="Actual Physical Drag |Fd|: 0.00 N")
        self.lbl_real_drag.pack(anchor=tk.W, pady=1)

        self.lbl_est_drag = ttk.Label(tele_frame, text="Observer Estimated Force |F_hat|: 0.00 N (Zero-Sensor)")
        self.lbl_est_drag.pack(anchor=tk.W, pady=1)

        self.lbl_obs_acc = ttk.Label(tele_frame, text="Observer Accuracy: 100.0%", style='Value.TLabel')
        self.lbl_obs_acc.pack(anchor=tk.W, pady=1)

        err_box = tk.Frame(tele_frame, bg='#1e1e24')
        err_box.pack(fill=tk.X, pady=(2, 0))
        ttk.Label(err_box, text="Position Tracking Error:").pack(side=tk.LEFT)
        self.lbl_tele_err = tk.Label(err_box, text="0.00 m (ROCK SOLID)", bg='#1e1e24', fg='#3fb950', font=('Helvetica', 10, 'bold'))
        self.lbl_tele_err.pack(side=tk.RIGHT)

        # ======================================================================
        # SECTION 5: BENCHMARK LOGGER
        # ======================================================================
        log_frame = ttk.LabelFrame(col_right, text=" 5. Data Logger & Performance Graphs ", padding="8")
        log_frame.pack(fill=tk.X, pady=3)

        self.btn_record = tk.Button(log_frame, text="⏺ START EXPERIMENT RECORDING", bg='#238636', fg='white',
                                    font=('Helvetica', 9, 'bold'), relief='flat', pady=4, command=self.toggle_recording)
        self.btn_record.pack(fill=tk.X, pady=2)

        self.btn_plot = tk.Button(log_frame, text="📊 Generate Performance Graphs", bg='#1f6feb', fg='white',
                                  font=('Helvetica', 9, 'bold'), relief='flat', pady=4, command=self.generate_plots)
        self.btn_plot.pack(fill=tk.X, pady=2)

        self.lbl_log_status = ttk.Label(log_frame, text="Status: Ready to log flight data")
        self.lbl_log_status.pack(pady=1)

    # -------------------------------------------------------------
    # Action Handlers
    # -------------------------------------------------------------
    def on_takeoff(self):
        self.ros_node.takeoff_pub.publish(Empty())
        self.lbl_log_status.config(text="Takeoff initiated: Lifting off to 2.0m Hover!")

    def on_land(self):
        self.ros_node.land_pub.publish(Empty())
        self.lbl_log_status.config(text="Landing initiated: Descending safely to ground.")

    def on_hover(self):
        self.ros_node.hover_pub.publish(Empty())
        self.active_keys.clear()
        self.ros_node.cmd_vel_pub.publish(Twist())
        self.lbl_log_status.config(text="Hover initiated: Holding current position.")

    def on_clear_path(self):
        self.ros_node.clear_path_pub.publish(Empty())
        self.lbl_log_status.config(text="Visual path trail cleared.")

    def on_tele_spd_change(self, _=None):
        spd = self.scale_tele_spd.get()
        self.teleop_speed = spd
        self.lbl_tele_spd.config(text=f"{spd:.2f} m/s")

    def pulse_teleop(self, vx=0.0, vy=0.0, vz=0.0, vyaw=0.0):
        t = Twist()
        t.linear.x = float(vx)
        t.linear.y = float(vy)
        t.linear.z = float(vz)
        t.angular.z = float(vyaw)
        self.ros_node.cmd_vel_pub.publish(t)

    def on_key_press(self, event):
        k = event.keysym.lower()
        if k in ['t']:
            self.on_takeoff()
        elif k in ['l']:
            self.on_land()
        elif k in ['h', 'space']:
            self.on_hover()
        elif k in ['c']:
            self.on_clear_path()
        else:
            self.active_keys.add(k)

    def on_key_release(self, event):
        k = event.keysym.lower()
        if k in self.active_keys:
            self.active_keys.remove(k)

    def teleop_publisher_loop(self):
        if self.active_keys:
            vx, vy, vz, vyaw = 0.0, 0.0, 0.0, 0.0
            if 'w' in self.active_keys or 'up' in self.active_keys:
                vx += self.teleop_speed
            if 's' in self.active_keys or 'down' in self.active_keys:
                vx -= self.teleop_speed
            if 'a' in self.active_keys or 'left' in self.active_keys:
                vy += self.teleop_speed
            if 'd' in self.active_keys or 'right' in self.active_keys:
                vy -= self.teleop_speed
            if 'i' in self.active_keys or 'e' in self.active_keys or 'prior' in self.active_keys:
                vz += self.teleop_climb_speed
            if 'k' in self.active_keys or 'q' in self.active_keys or 'next' in self.active_keys:
                vz -= self.teleop_climb_speed
            if 'j' in self.active_keys:
                vyaw += self.teleop_yaw_speed
            if 'semicolon' in self.active_keys or 'bracketright' in self.active_keys:
                vyaw -= self.teleop_yaw_speed

            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            twist.linear.z = vz
            twist.angular.z = vyaw
            self.ros_node.cmd_vel_pub.publish(twist)

        self.root.after(50, self.teleop_publisher_loop)

    def on_flight_speed_change(self, _=None):
        spd = self.scale_flight_speed.get()
        self.lbl_flight_speed.config(text=f"{spd:.2f} m/s")
        msg = Float32()
        msg.data = float(spd)
        self.ros_node.speed_pub.publish(msg)

    def on_flight_heading_change(self, _=None):
        deg = self.scale_flight_dir.get()
        cardinal = "East"
        if 45 <= deg < 135:
            cardinal = "North"
        elif 135 <= deg < 225:
            cardinal = "West"
        elif 225 <= deg < 315:
            cardinal = "South"
        self.lbl_flight_dir.config(text=f"{deg:.0f} deg ({cardinal})")
        msg = Float32()
        msg.data = float(deg)
        self.ros_node.heading_pub.publish(msg)

    def on_ctrl_mode_change(self):
        val = self.ctrl_mode_var.get()
        msg = Bool()
        msg.data = val
        self.ros_node.dob_pub.publish(msg)

    def on_wind_change(self, _=None):
        spd = self.scale_wind_speed.get()
        deg = self.scale_wind_dir.get()
        turb = self.scale_turb.get()

        self.lbl_wind_speed.config(text=f"{spd:.1f} m/s")
        self.lbl_wind_dir.config(text=f"{deg:.0f} deg")
        self.lbl_turb.config(text=f"{turb*100:.0f}%")

        msg = Vector3()
        msg.x = float(spd)
        msg.y = float(deg)
        msg.z = float(turb)
        self.ros_node.wind_pub.publish(msg)

    def trigger_gust(self):
        msg = Float32()
        msg.data = 8.0
        self.ros_node.gust_pub.publish(msg)

    def reset_calm(self):
        self.scale_wind_speed.set(0.0)
        self.scale_turb.set(0.0)
        self.on_wind_change()

    def on_traj_change(self):
        val = self.traj_var.get()
        msg = Int32()
        msg.data = val
        self.ros_node.traj_pub.publish(msg)

    def toggle_recording(self):
        if not self.recording:
            self.recording = True
            self.log_data = []
            self.record_start_time = time.time()
            self.btn_record.config(text="⏹ STOP & SAVE LOG", bg='#da3633')
            self.lbl_log_status.config(text="Logging: Active (Recording samples...)")
        else:
            self.recording = False
            self.btn_record.config(text="⏺ START EXPERIMENT RECORDING", bg='#238636')
            filepath = os.path.expanduser("~/ros2_uav_ws/experiment_flight_log.csv")
            self.save_csv(filepath)
            self.lbl_log_status.config(text=f"Saved {len(self.log_data)} samples to experiment_flight_log.csv")

    def save_csv(self, filepath):
        header = ['time_sec', 'pos_x', 'pos_y', 'pos_z', 'vel_x', 'vel_y', 'vel_z',
                  'wind_x', 'wind_y', 'wind_z', 'drag_x', 'drag_y', 'drag_z', 'error_x', 'error_y', 'error_z', 'error_norm']
        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(self.log_data)

    def generate_plots(self):
        filepath = os.path.expanduser("~/ros2_uav_ws/experiment_flight_log.csv")
        plot_script = os.path.expanduser("~/ros2_uav_ws/src/uav_dashboard/uav_dashboard/plot_performance.py")
        if os.path.exists(filepath):
            os.system(f"python3 {plot_script} {filepath} &")
        else:
            messagebox.showwarning("No Data", "Please record an experiment flight first by clicking 'START EXPERIMENT RECORDING'!")

    def update_telemetry_loop(self):
        with self.ros_node.lock:
            p = list(self.ros_node.pos)
            v = list(self.ros_node.vel)
            w = list(self.ros_node.wind)
            d = list(self.ros_node.drag)
            ed = list(self.ros_node.est_drag)
            e = list(self.ros_node.error)
            st = self.ros_node.flight_status

        spd = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
        w_spd = math.sqrt(w[0]**2 + w[1]**2 + w[2]**2)
        d_mag = math.sqrt(d[0]**2 + d[1]**2 + d[2]**2)
        ed_mag = math.sqrt(ed[0]**2 + ed[1]**2 + ed[2]**2)
        err_mag = math.sqrt(e[0]**2 + e[1]**2 + e[2]**2)

        # Status text & color
        if st == "LANDED":
            self.lbl_flight_state.config(text="FLIGHT STATUS: ● ON GROUND (LANDED)", fg='#e3b341')
        elif st == "TAKEOFF":
            self.lbl_flight_state.config(text="FLIGHT STATUS: ▲ TAKING OFF (CLIMBING)...", fg='#58a6ff')
        elif st == "HOVER":
            self.lbl_flight_state.config(text=f"FLIGHT STATUS: ◆ HOVERING ({p[2]:.1f}m)", fg='#3fb950')
        elif st == "TELEOP":
            self.lbl_flight_state.config(text=f"FLIGHT STATUS: ▶ TELEOP ACTIVE ({spd:.1f} m/s)", fg='#7ee787')
        elif st == "LANDING":
            self.lbl_flight_state.config(text="FLIGHT STATUS: ▼ LANDING (DESCENDING)...", fg='#f85149')
        else:
            self.lbl_flight_state.config(text=f"FLIGHT STATUS: {st}", fg='#58a6ff')

        self.lbl_tele_pos.config(text=f"Position: [{p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}] m")
        self.lbl_tele_vel.config(text=f"Ground Speed: {spd:.2f} m/s")
        self.lbl_tele_wind.config(text=f"Wind Velocity: {w_spd:.2f} m/s")
        self.lbl_real_drag.config(text=f"Actual Physical Drag |Fd|: {d_mag:.2f} N")
        self.lbl_est_drag.config(text=f"Observer Estimated Force |F_hat|: {ed_mag:.2f} N (Zero-Sensor)")

        if d_mag > 0.05:
            acc = max(0.0, 100.0 * (1.0 - abs(d_mag - ed_mag) / d_mag))
            self.lbl_obs_acc.config(text=f"Observer Accuracy: {acc:.1f}%")
        else:
            self.lbl_obs_acc.config(text="Observer Accuracy: 100.0% (Calm)")

        if err_mag < 0.05:
            self.lbl_tele_err.config(text=f"{err_mag:.2f} m (ROCK SOLID - ZERO DRIFT)", fg='#3fb950')
        elif err_mag < 0.25:
            self.lbl_tele_err.config(text=f"{err_mag:.2f} m (MINOR DEFLECTION)", fg='#d29922')
        else:
            self.lbl_tele_err.config(text=f"{err_mag:.2f} m (LARGE DRIFT)", fg='#f85149')

        if self.recording:
            t = time.time() - self.record_start_time
            self.log_data.append([
                f"{t:.3f}", f"{p[0]:.3f}", f"{p[1]:.3f}", f"{p[2]:.3f}",
                f"{v[0]:.3f}", f"{v[1]:.3f}", f"{v[2]:.3f}",
                f"{w[0]:.3f}", f"{w[1]:.3f}", f"{w[2]:.3f}",
                f"{d[0]:.3f}", f"{d[1]:.3f}", f"{d[2]:.3f}",
                f"{e[0]:.3f}", f"{e[1]:.3f}", f"{e[2]:.3f}", f"{err_mag:.3f}"
            ])

        self.root.after(50, self.update_telemetry_loop)

def main(args=None):
    rclpy.init(args=args)
    ros_node = RosGuiBridge()

    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    root = tk.Tk()
    app = LiveTuningApp(root, ros_node)

    try:
        root.mainloop()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
