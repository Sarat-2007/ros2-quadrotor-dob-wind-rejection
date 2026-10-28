#!/usr/bin/env python3
"""
Official UAV Ground Control Station (GCS) Dashboard & Flight Controller
Design Standard: Industrial Aerospace & Defense GCS (Neutral Dark Slate Palette)
Features:
- Primary Flight Control: Takeoff, Position Hold, Land
- Continuous Manual Flight Stick (D-Pad + Keyboard W/A/S/D, I/K, J/;)
- Disturbance Observer (DOB / ADRC) Zero-Drift Mode Toggle
- Aerodynamic Crosswind & 8 m/s Gust Injection
- Live Real-Time Telemetry HUD (Altitude, Ground Speed, 3D Position, Heading, Est. Disturbance)
- No Gradients, No Neon/Purple/Pink Styling - Clean Industrial Engineering UI
"""

import sys
import math
import numpy as np
import tkinter as tk
from tkinter import ttk
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, WrenchStamped, PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String, Bool, Float32, Int32

class GcsRosBridge(Node):
    def __init__(self):
        super().__init__('uav_gcs_node')

        # Command Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.takeoff_pub = self.create_publisher(Empty, '/uav/cmd_takeoff', 10)
        self.land_pub = self.create_publisher(Empty, '/uav/cmd_land', 10)
        self.hover_pub = self.create_publisher(Empty, '/uav/cmd_hover', 10)
        self.clear_path_pub = self.create_publisher(Empty, '/uav/clear_path', 10)
        self.reset_home_pub = self.create_publisher(Empty, '/uav/reset_to_home', 10)
        self.dob_pub = self.create_publisher(Bool, '/uav/set_dob_mode', 10)
        self.wind_pub = self.create_publisher(Vector3, '/uav/set_wind', 10)
        self.gust_pub = self.create_publisher(Float32, '/uav/trigger_gust', 10)
        self.traj_mode_pub = self.create_publisher(Int32, '/uav/set_trajectory_mode', 10)
        self.setpoint_pub = self.create_publisher(PoseStamped, '/uav/setpoint', 10)
        self.setpoint_vel_pub = self.create_publisher(Vector3Stamped, '/uav/setpoint_velocity', 10)

        # Telemetry State
        self.lock = threading.Lock()
        self.pos = [0.0, 0.0, 0.12]
        self.vel = [0.0, 0.0, 0.0]
        self.yaw = 0.0
        self.status = "LANDED"
        self.est_dist = [0.0, 0.0, 0.0]
        self.dob_active = True

        # Subscribers
        self.create_subscription(Odometry, '/uav/odom', self.odom_cb, 10)
        self.create_subscription(String, '/uav/flight_status', self.status_cb, 10)
        self.create_subscription(WrenchStamped, '/uav/estimated_disturbance', self.dist_cb, 10)

    def odom_cb(self, msg: Odometry):
        with self.lock:
            self.pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
            self.vel = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def status_cb(self, msg: String):
        with self.lock:
            self.status = msg.data

    def dist_cb(self, msg: WrenchStamped):
        with self.lock:
            self.est_dist = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]

class OfficialGcsDashboard:
    def __init__(self, root, ros_node: GcsRosBridge):
        self.root = root
        self.ros_node = ros_node
        self.root.title("UAV GROUND CONTROL STATION | Flight Autopilot & Telemetry")
        self.root.geometry("580x870")
        self.root.configure(bg='#1a1d24')
        self.root.resizable(False, False)

        # Flight velocity parameters
        self.teleop_speed = 1.2
        self.teleop_climb_speed = 0.8
        self.teleop_yaw_speed = 0.8

        self.active_keys = set()
        self._release_timers = {}
        self._is_streaming = False

        # Autonomous Surveillance Patrol Mission State
        self.patrol_active = False
        self.patrol_waiting_for_takeoff = False
        self.patrol_wp_idx = 0
        self.patrol_target_pos = np.array([0.0, 0.0, 2.0])
        self.patrol_speed = 1.2
        self.patrol_waypoints = [
            {"name": "Helipad Transit Corridor", "pos": [0.0, 1.5, 2.0], "desc": "Climb out & align with gate entry"},
            {"name": "Inspection Gate Ingress", "pos": [0.0, 5.2, 2.2], "desc": "Fly through illuminated gate center"},
            {"name": "East Sector Turn", "pos": [2.2, 4.5, 2.5], "desc": "Clear past Slalom Pylon 1"},
            {"name": "Rooftop Observation Deck", "pos": [5.0, 0.5, 2.7], "desc": "Overfly elevated rooftop structure"},
            {"name": "Industrial Tower A Perimeter", "pos": [5.5, -4.0, 2.6], "desc": "Perimeter inspection of Tower A"},
            {"name": "South Sector Cross-Corridor", "pos": [2.5, -5.8, 2.4], "desc": "Transit between Industrial Towers"},
            {"name": "Industrial Tower B Perimeter", "pos": [-3.5, -5.5, 2.4], "desc": "Perimeter inspection of Tower B"},
            {"name": "West Sector Approach", "pos": [-3.5, -1.5, 2.2], "desc": "Fly inbound past Slalom Pylon 3"},
            {"name": "Helipad Home Recovery", "pos": [0.0, 0.0, 2.0], "desc": "Return to base & lock station hover"}
        ]

        self.setup_ui()
        self.bind_keyboard()
        self.update_telemetry_loop()
        self.teleop_stream_loop()
        self.patrol_stream_loop()

        self.root.focus_force()

    def bind_keyboard(self):
        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.bind('<KeyRelease>', self.on_key_release)

    def bind_dpad_hold(self, widget, key_code):
        widget.bind('<ButtonPress-1>', lambda e: self.on_gui_btn_down(key_code))
        widget.bind('<ButtonRelease-1>', lambda e: self.on_gui_btn_up(key_code))

    def on_gui_btn_down(self, key_code):
        if self.patrol_active:
            self.do_abort_patrol(reason="PILOT STICK OVERRIDE")
        self.active_keys.add(key_code)
        self.update_key_button_highlight(key_code, is_pressed=True)

    def on_gui_btn_up(self, key_code):
        if key_code in self.active_keys:
            self.active_keys.remove(key_code)
        self.update_key_button_highlight(key_code, is_pressed=False)

    def setup_ui(self):
        # 1. Official Header
        header = tk.Frame(self.root, bg='#242933', bd=1, relief='solid', padx=12, pady=8)
        header.pack(fill=tk.X, padx=12, pady=(10, 6))

        title_lbl = tk.Label(header, text="UAV GROUND CONTROL STATION", font=('Helvetica', 12, 'bold'),
                             bg='#242933', fg='#e6edf3')
        title_lbl.pack(anchor='w')

        sub_lbl = tk.Label(header, text="Autonomous & Teleoperated Quadrotor Flight System",
                           font=('Helvetica', 9), bg='#242933', fg='#8b949e')
        sub_lbl.pack(anchor='w')

        self.lbl_status_banner = tk.Label(header, text="● SYSTEM STANDBY: ON GROUND SKIDS (Z = 0.12m)",
                                          font=('Helvetica', 10, 'bold'), bg='#242933', fg='#d29922')
        self.lbl_status_banner.pack(anchor='w', pady=(6, 2))

        # 2. Master Primary Flight Actions
        action_frame = tk.Frame(self.root, bg='#1a1d24')
        action_frame.pack(fill=tk.X, padx=12, pady=4)

        self.btn_takeoff = tk.Button(action_frame, text="▲ ARM & TAKEOFF\n[Key: T]", bg='#238636', fg='#ffffff',
                                     font=('Helvetica', 10, 'bold'), relief='flat', height=2,
                                     activebackground='#2ea043', command=self.do_takeoff)
        self.btn_takeoff.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        self.btn_hover = tk.Button(action_frame, text="■ POSITION HOLD\n[Key: H / Space]", bg='#1f6feb', fg='#ffffff',
                                   font=('Helvetica', 10, 'bold'), relief='flat', height=2,
                                   activebackground='#388bfd', command=self.do_hover)
        self.btn_hover.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        self.btn_land = tk.Button(action_frame, text="▼ LAND & DISARM\n[Key: L]", bg='#892222', fg='#ffffff',
                                  font=('Helvetica', 10, 'bold'), relief='flat', height=2,
                                  activebackground='#a82828', command=self.do_land)
        self.btn_land.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        # 3. Autonomous Surveillance Mission Control Panel
        mission_box = tk.LabelFrame(self.root, text=" Autonomous Surveillance Mission (3D Obstacle Arena) ",
                                    font=('Helvetica', 9, 'bold'), bg='#242933', fg='#c9d1d9', padx=10, pady=6)
        mission_box.pack(fill=tk.X, padx=12, pady=4)

        m_btns = tk.Frame(mission_box, bg='#242933')
        m_btns.pack(fill=tk.X, pady=2)

        self.btn_start_patrol = tk.Button(m_btns, text="🎯 START AUTONOMOUS PATROL\n[Gate & Towers Perimeter Sweep]",
                                          bg='#238636', fg='#ffffff', font=('Helvetica', 9, 'bold'),
                                          relief='flat', height=2, activebackground='#2ea043',
                                          command=self.do_start_patrol)
        self.btn_start_patrol.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        self.btn_abort_patrol = tk.Button(m_btns, text="🛑 ABORT PATROL MISSION\n[Instant Station Hover]",
                                          bg='#892222', fg='#ffffff', font=('Helvetica', 9, 'bold'),
                                          relief='flat', height=2, activebackground='#a82828',
                                          command=self.do_abort_patrol)
        self.btn_abort_patrol.pack(side=tk.RIGHT, expand=True, fill=tk.X, padx=2)

        m_status = tk.Frame(mission_box, bg='#1a1d24', padx=8, pady=4)
        m_status.pack(fill=tk.X, pady=(4, 2))

        self.lbl_patrol_status = tk.Label(m_status, text="MISSION STANDBY: Ready to deploy across 9 facility waypoints",
                                          bg='#1a1d24', fg='#8b949e', font=('Helvetica', 9, 'bold'))
        self.lbl_patrol_status.pack(anchor='w')

        self.lbl_patrol_sub = tk.Label(m_status, text="Route: Helipad → Gate (0,5m) → Rooftop → Tower A → Tower B → Helipad",
                                       bg='#1a1d24', fg='#58a6ff', font=('Helvetica', 8))
        self.lbl_patrol_sub.pack(anchor='w')

        # 4. Manual Flight Stick (D-Pad)
        dpad_box = tk.LabelFrame(self.root, text=" Manual Flight Control (Hold to Move) ",
                                 font=('Helvetica', 9, 'bold'), bg='#242933', fg='#c9d1d9', padx=10, pady=8)
        dpad_box.pack(fill=tk.X, padx=12, pady=6)

        # Row 1: Forward & Climb
        r1 = tk.Frame(dpad_box, bg='#242933')
        r1.pack(pady=2)
        self.btn_fwd = tk.Button(r1, text="↑ FORWARD [W / Up]", width=16, height=2,
                                 bg='#30363d', fg='#e6edf3', font=('Helvetica', 9), relief='groove')
        self.btn_fwd.pack(side=tk.LEFT, padx=3)
        self.bind_dpad_hold(self.btn_fwd, 'w')

        self.btn_climb = tk.Button(r1, text="▲ CLIMB [I / E]", width=14, height=2,
                                  bg='#30363d', fg='#e6edf3', font=('Helvetica', 9), relief='groove')
        self.btn_climb.pack(side=tk.LEFT, padx=3)
        self.bind_dpad_hold(self.btn_climb, 'i')

        # Row 2: Left, Brake, Right
        r2 = tk.Frame(dpad_box, bg='#242933')
        r2.pack(pady=2)
        self.btn_left = tk.Button(r2, text="← LEFT [A / Left]", width=14, height=2,
                                  bg='#30363d', fg='#e6edf3', font=('Helvetica', 9), relief='groove')
        self.btn_left.pack(side=tk.LEFT, padx=3)
        self.bind_dpad_hold(self.btn_left, 'a')

        self.btn_stop = tk.Button(r2, text="HOLD\n[Space]", width=10, height=2,
                                  bg='#1f6feb', fg='#ffffff', font=('Helvetica', 9, 'bold'), relief='flat',
                                  command=self.do_hover)
        self.btn_stop.pack(side=tk.LEFT, padx=3)

        self.btn_right = tk.Button(r2, text="→ RIGHT [D / Right]", width=14, height=2,
                                   bg='#30363d', fg='#e6edf3', font=('Helvetica', 9), relief='groove')
        self.btn_right.pack(side=tk.LEFT, padx=3)
        self.bind_dpad_hold(self.btn_right, 'd')

        # Row 3: Backward & Descend
        r3 = tk.Frame(dpad_box, bg='#242933')
        r3.pack(pady=2)
        self.btn_back = tk.Button(r3, text="↓ BACKWARD [S / Down]", width=16, height=2,
                                  bg='#30363d', fg='#e6edf3', font=('Helvetica', 9), relief='groove')
        self.btn_back.pack(side=tk.LEFT, padx=3)
        self.bind_dpad_hold(self.btn_back, 's')

        self.btn_descend = tk.Button(r3, text="▼ DESCEND [K / Q]", width=14, height=2,
                                     bg='#30363d', fg='#e6edf3', font=('Helvetica', 9), relief='groove')
        self.btn_descend.pack(side=tk.LEFT, padx=3)
        self.bind_dpad_hold(self.btn_descend, 'k')

        # Row 4: Yaw Left & Yaw Right
        r4 = tk.Frame(dpad_box, bg='#242933')
        r4.pack(pady=2)
        self.btn_yaw_l = tk.Button(r4, text="↺ Turn Left [J]", width=15, bg='#30363d', fg='#e6edf3',
                                   font=('Helvetica', 9), relief='groove')
        self.btn_yaw_l.pack(side=tk.LEFT, padx=3)
        self.bind_dpad_hold(self.btn_yaw_l, 'j')

        self.btn_yaw_r = tk.Button(r4, text="↻ Turn Right [;]", width=15, bg='#30363d', fg='#e6edf3',
                                   font=('Helvetica', 9), relief='groove')
        self.btn_yaw_r.pack(side=tk.LEFT, padx=3)
        self.bind_dpad_hold(self.btn_yaw_r, 'semicolon')

        # Speed scale
        spd_row = tk.Frame(dpad_box, bg='#242933')
        spd_row.pack(fill=tk.X, pady=(6, 2))
        tk.Label(spd_row, text="Teleop Flight Speed:", bg='#242933', fg='#8b949e', font=('Helvetica', 9)).pack(side=tk.LEFT)
        self.lbl_speed_val = tk.Label(spd_row, text="1.20 m/s", bg='#242933', fg='#58a6ff', font=('Helvetica', 9, 'bold'))
        self.lbl_speed_val.pack(side=tk.RIGHT)
        self.scale_speed = ttk.Scale(dpad_box, from_=0.3, to=3.0, value=1.2, command=self.on_speed_slide)
        self.scale_speed.pack(fill=tk.X)

        # 4. Control Architecture & Disturbance Observer (Official Mode Panel)
        ctrl_box = tk.LabelFrame(self.root, text=" Autopilot & Disturbance Observer Configuration ",
                                 font=('Helvetica', 9, 'bold'), bg='#242933', fg='#c9d1d9', padx=10, pady=8)
        ctrl_box.pack(fill=tk.X, padx=12, pady=6)

        c1 = tk.Frame(ctrl_box, bg='#242933')
        c1.pack(fill=tk.X, pady=2)
        tk.Label(c1, text="Disturbance Observer (DOB):", bg='#242933', fg='#8b949e', font=('Helvetica', 9)).pack(side=tk.LEFT)
        self.btn_dob_toggle = tk.Button(c1, text="DOB ACTIVE (Zero-Drift Wind Rejection)", bg='#238636', fg='#ffffff',
                                        font=('Helvetica', 9, 'bold'), relief='flat', padx=8, command=self.toggle_dob)
        self.btn_dob_toggle.pack(side=tk.RIGHT)

        # Dynamic Gust Velocity Setting & Real-Time Aerodynamic Calculator
        c_gust_lbl = tk.Frame(ctrl_box, bg='#242933')
        c_gust_lbl.pack(fill=tk.X, pady=(8, 2))
        tk.Label(c_gust_lbl, text="Shock Gust Velocity (v):", bg='#242933', fg='#8b949e', font=('Helvetica', 9)).pack(side=tk.LEFT)
        self.lbl_gust_val = tk.Label(c_gust_lbl, text="8.0 m/s", bg='#242933', fg='#58a6ff', font=('Helvetica', 9, 'bold'))
        self.lbl_gust_val.pack(side=tk.RIGHT)

        c_gust_slide = tk.Frame(ctrl_box, bg='#242933')
        c_gust_slide.pack(fill=tk.X, pady=2)
        self.scale_gust = ttk.Scale(c_gust_slide, from_=1.0, to=12.0, value=8.0, command=self.on_gust_slide)
        self.scale_gust.pack(fill=tk.X)

        # Real-Time Calculated Physics Display
        c_calc = tk.Frame(ctrl_box, bg='#1a1d24', padx=8, pady=5)
        c_calc.pack(fill=tk.X, pady=4)
        self.lbl_calc_drag = tk.Label(c_calc, text="Fluid Drag Force (Fd = ½ρCdAv²): 9.92 N", bg='#1a1d24', fg='#58a6ff', font=('Helvetica', 8, 'bold'))
        self.lbl_calc_drag.pack(anchor=tk.W)
        self.lbl_calc_tilt = tk.Label(c_calc, text="Req. Counter-Tilt: 35.1° | Collective Thrust: 18.01 N", bg='#1a1d24', fg='#8b949e', font=('Helvetica', 8))
        self.lbl_calc_tilt.pack(anchor=tk.W)

        # Inject Button with Real-Time Values
        self.btn_inject_gust = tk.Button(ctrl_box, text="⚡ INJECT 8.0 m/s GUST (Exp. Force: 9.92 N)", bg='#30363d', fg='#ffffff',
                                         font=('Helvetica', 9, 'bold'), relief='flat', pady=4, command=self.do_trigger_gust)
        self.btn_inject_gust.pack(fill=tk.X, pady=(2, 4))

        # Quick Shock Gust Presets
        c_presets = tk.Frame(ctrl_box, bg='#242933')
        c_presets.pack(fill=tk.X, pady=(2, 4))
        tk.Label(c_presets, text="Quick Presets:", bg='#242933', fg='#8b949e', font=('Helvetica', 8)).pack(side=tk.LEFT)
        for g_val in [4.0, 6.0, 8.0, 10.0]:
            tk.Button(c_presets, text=f"{int(g_val)} m/s", bg='#21262d', fg='#e6edf3', font=('Helvetica', 8),
                      relief='flat', command=lambda v=g_val: self.set_gust_preset(v)).pack(side=tk.LEFT, padx=3)

        # Ambient Continuous Wind Slider
        c_amb = tk.Frame(ctrl_box, bg='#242933')
        c_amb.pack(fill=tk.X, pady=(6, 2))
        tk.Label(c_amb, text="Continuous Ambient Wind:", bg='#242933', fg='#8b949e', font=('Helvetica', 8)).pack(side=tk.LEFT)
        self.lbl_wind_val = tk.Label(c_amb, text="0.0 m/s", bg='#242933', fg='#8b949e', font=('Helvetica', 8, 'bold'))
        self.lbl_wind_val.pack(side=tk.RIGHT)

        c_amb_slide = tk.Frame(ctrl_box, bg='#242933')
        c_amb_slide.pack(fill=tk.X, pady=2)
        self.scale_wind = ttk.Scale(c_amb_slide, from_=0.0, to=8.0, value=0.0, command=self.on_wind_slide)
        self.scale_wind.pack(fill=tk.X)

        # 5. Live Telemetry HUD Table
        hud_box = tk.LabelFrame(self.root, text=" Live Flight Telemetry ",
                                font=('Helvetica', 9, 'bold'), bg='#242933', fg='#c9d1d9', padx=10, pady=8)
        hud_box.pack(fill=tk.X, padx=12, pady=6)

        t1 = tk.Frame(hud_box, bg='#242933')
        t1.pack(fill=tk.X, pady=2)
        self.lbl_alt = tk.Label(t1, text="Altitude (Z): 0.12 m", font=('Helvetica', 10, 'bold'), bg='#242933', fg='#e6edf3')
        self.lbl_alt.pack(side=tk.LEFT)
        self.lbl_vel = tk.Label(t1, text="Ground Speed: 0.00 m/s", font=('Helvetica', 10, 'bold'), bg='#242933', fg='#58a6ff')
        self.lbl_vel.pack(side=tk.RIGHT)

        t2 = tk.Frame(hud_box, bg='#242933')
        t2.pack(fill=tk.X, pady=2)
        self.lbl_pos = tk.Label(t2, text="Position [X, Y]: [0.00, 0.00] m", font=('Helvetica', 9), bg='#242933', fg='#8b949e')
        self.lbl_pos.pack(side=tk.LEFT)
        self.lbl_yaw = tk.Label(t2, text="Heading: 0.0 deg", font=('Helvetica', 9), bg='#242933', fg='#8b949e')
        self.lbl_yaw.pack(side=tk.RIGHT)

        t3 = tk.Frame(hud_box, bg='#242933')
        t3.pack(fill=tk.X, pady=2)
        self.lbl_drag = tk.Label(t3, text="Estimated Disturbance Drag: 0.00 N", font=('Helvetica', 9), bg='#242933', fg='#8b949e')
        self.lbl_drag.pack(side=tk.LEFT)

        # 6. Utility Bar
        util_bar = tk.Frame(self.root, bg='#1a1d24')
        util_bar.pack(fill=tk.X, padx=12, pady=(4, 10))

        tk.Button(util_bar, text="Clear Flight Trail [C]", bg='#21262d', fg='#8b949e', font=('Helvetica', 8, 'bold'),
                  relief='flat', command=self.do_clear_path).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        tk.Button(util_bar, text="Reset Origin (0,0,2m)", bg='#21262d', fg='#58a6ff', font=('Helvetica', 8, 'bold'),
                  relief='flat', command=self.do_reset_home).pack(side=tk.RIGHT, expand=True, fill=tk.X, padx=2)

    # -------------------------------------------------------------
    # Action Functions
    # -------------------------------------------------------------
    def do_takeoff(self):
        self.ros_node.takeoff_pub.publish(Empty())

    def do_land(self):
        if self.patrol_active:
            self.do_abort_patrol(reason="LAND COMMANDED")
        self.ros_node.land_pub.publish(Empty())

    def do_hover(self):
        if self.patrol_active:
            self.do_abort_patrol(reason="POSITION HOLD COMMANDED")
        for t in self._release_timers.values():
            self.root.after_cancel(t)
        self._release_timers.clear()
        self.active_keys.clear()
        self._is_streaming = False
        self.ros_node.hover_pub.publish(Empty())
        self.ros_node.cmd_vel_pub.publish(Twist())
        self.reset_button_colors()

    def do_clear_path(self):
        self.ros_node.clear_path_pub.publish(Empty())

    def do_reset_home(self):
        if self.patrol_active:
            self.do_abort_patrol(reason="RESET HOME COMMANDED")
        self.ros_node.reset_home_pub.publish(Empty())

    # -------------------------------------------------------------
    # Autonomous Surveillance Mission Control
    # -------------------------------------------------------------
    def do_start_patrol(self):
        if self.patrol_active:
            return
        with self.ros_node.lock:
            st = self.ros_node.status
            current_pos = list(self.ros_node.pos)

        self.patrol_active = True
        self.patrol_wp_idx = 0
        self.btn_start_patrol.config(text="● PATROL ACTIVE (SURVEILLANCE)...", bg='#1f6feb')

        if st == "LANDED" or current_pos[2] < 0.5:
            self.patrol_waiting_for_takeoff = True
            self.ros_node.takeoff_pub.publish(Empty())
            self.lbl_patrol_status.config(text="🚀 TAKEOFF CLIMB INITIATED: Climbing to safe transit altitude...", fg='#d29922')
            self.lbl_patrol_sub.config(text="Waypoint tracking will commence automatically at 2.0 m.", fg='#8b949e')
        else:
            self.patrol_waiting_for_takeoff = False
            self.patrol_target_pos = np.array(current_pos, dtype=float)
            msg = Int32()
            msg.data = 10
            self.ros_node.traj_mode_pub.publish(msg)
            self.lbl_patrol_status.config(text="🎯 AUTONOMOUS PATROL INITIATED: Tracking Waypoint 1", fg='#3fb950')
            self.lbl_patrol_sub.config(text="Full 3D perimeter surveillance across inspection gate and industrial towers.", fg='#58a6ff')

    def do_abort_patrol(self, reason="PILOT ABORT"):
        if not self.patrol_active:
            self.do_hover()
            return
        self.patrol_active = False
        self.patrol_waiting_for_takeoff = False
        msg = Int32()
        msg.data = 0
        self.ros_node.traj_mode_pub.publish(msg)
        self.ros_node.hover_pub.publish(Empty())
        self.ros_node.cmd_vel_pub.publish(Twist())
        self.btn_start_patrol.config(text="🎯 START AUTONOMOUS PATROL\n[Gate & Towers Perimeter Sweep]", bg='#238636')
        self.lbl_patrol_status.config(text=f"⚠️ PATROL ABORTED: {reason}", fg='#f85149')
        self.lbl_patrol_sub.config(text="Station hover locked at current coordinates. Pilot sticks active.", fg='#e6edf3')

    def patrol_stream_loop(self):
        if self.patrol_active:
            with self.ros_node.lock:
                current_pos = np.array(self.ros_node.pos, dtype=float)
                current_status = self.ros_node.status

            if self.patrol_waiting_for_takeoff:
                if current_pos[2] >= 1.80:
                    self.patrol_waiting_for_takeoff = False
                    self.patrol_wp_idx = 0
                    self.patrol_target_pos = np.copy(current_pos)
                    msg = Int32()
                    msg.data = 10
                    self.ros_node.traj_mode_pub.publish(msg)
                    self.lbl_patrol_status.config(text="✅ ALTITUDE REACHED: Engaging Waypoint 1 (Inspection Gate)", fg='#3fb950')
                else:
                    self.lbl_patrol_status.config(
                        text=f"🚀 TAKEOFF CLIMB: {current_pos[2]:.2f} / 2.00 m (Ascending)",
                        fg='#d29922'
                    )
            else:
                if self.patrol_wp_idx < len(self.patrol_waypoints):
                    wp = self.patrol_waypoints[self.patrol_wp_idx]
                    wp_pos = np.array(wp["pos"], dtype=float)
                    d_vec = wp_pos - current_pos
                    dist_to_wp = float(np.linalg.norm(d_vec))

                    # Arrival tolerance: 0.40m
                    if dist_to_wp < 0.40:
                        self.patrol_wp_idx += 1
                        if self.patrol_wp_idx >= len(self.patrol_waypoints):
                            # Mission successfully finished!
                            self.patrol_active = False
                            msg = Int32()
                            msg.data = 0
                            self.ros_node.traj_mode_pub.publish(msg)
                            self.ros_node.hover_pub.publish(Empty())
                            self.btn_start_patrol.config(text="🎯 START AUTONOMOUS PATROL\n[Gate & Towers Perimeter Sweep]", bg='#238636')
                            self.lbl_patrol_status.config(text="✅ MISSION COMPLETE: RECOVERED TO BASE HELIPAD", fg='#3fb950')
                            self.lbl_patrol_sub.config(text="All 9 surveillance waypoints traversed. Holding position at (0, 0, 2m).", fg='#58a6ff')
                            self.root.after(50, self.patrol_stream_loop)
                            return
                        else:
                            wp = self.patrol_waypoints[self.patrol_wp_idx]
                            wp_pos = np.array(wp["pos"], dtype=float)
                            d_vec = wp_pos - current_pos
                            dist_to_wp = float(np.linalg.norm(d_vec))

                    # Advance virtual carrot towards waypoint at cruising speed
                    dt = 0.05
                    step = self.patrol_speed * dt
                    carrot_vec = wp_pos - self.patrol_target_pos
                    dist_carrot = float(np.linalg.norm(carrot_vec))
                    if dist_carrot > step:
                        self.patrol_target_pos += (carrot_vec / dist_carrot) * step
                    else:
                        self.patrol_target_pos = np.copy(wp_pos)

                    # Clamp carrot if drone is lagging behind (e.g. during strong wind gust)
                    drone_to_carrot = self.patrol_target_pos - current_pos
                    lead_dist = float(np.linalg.norm(drone_to_carrot))
                    if lead_dist > 0.65:
                        self.patrol_target_pos = current_pos + (drone_to_carrot / lead_dist) * 0.65

                    # Publish setpoint to flight controller
                    sp_msg = PoseStamped()
                    sp_msg.header.stamp = self.ros_node.get_clock().now().to_msg()
                    sp_msg.header.frame_id = 'world'
                    sp_msg.pose.position.x = float(self.patrol_target_pos[0])
                    sp_msg.pose.position.y = float(self.patrol_target_pos[1])
                    sp_msg.pose.position.z = float(self.patrol_target_pos[2])

                    # Calculate heading orientation towards target waypoint
                    target_yaw = math.atan2(d_vec[1], d_vec[0])
                    sp_msg.pose.orientation.w = float(math.cos(target_yaw * 0.5))
                    sp_msg.pose.orientation.z = float(math.sin(target_yaw * 0.5))
                    self.ros_node.setpoint_pub.publish(sp_msg)

                    # Update UI progress
                    pct = int(((self.patrol_wp_idx) / len(self.patrol_waypoints)) * 100)
                    self.lbl_patrol_status.config(
                        text=f"PATROL WP {self.patrol_wp_idx + 1}/{len(self.patrol_waypoints)}: {wp['name']} ({dist_to_wp:.1f}m | {pct}%)",
                        fg='#58a6ff'
                    )
                    self.lbl_patrol_sub.config(
                        text=f"Target: [{wp_pos[0]:.1f}, {wp_pos[1]:.1f}, {wp_pos[2]:.1f}] m | {wp['desc']}",
                        fg='#8b949e'
                    )

        self.root.after(50, self.patrol_stream_loop)

    def on_speed_slide(self, _=None):
        self.teleop_speed = self.scale_speed.get()
        self.lbl_speed_val.config(text=f"{self.teleop_speed:.2f} m/s")

    def on_wind_slide(self, _=None):
        w_val = self.scale_wind.get()
        self.lbl_wind_val.config(text=f"{w_val:.1f} m/s")
        msg = Vector3()
        msg.x = float(w_val)
        msg.y = 0.0
        msg.z = 0.0
        self.ros_node.wind_pub.publish(msg)

    def on_gust_slide(self, _=None):
        v = self.scale_gust.get()
        self.lbl_gust_val.config(text=f"{v:.1f} m/s")
        self.update_gust_calculations(v)

    def set_gust_preset(self, val):
        self.scale_gust.set(val)
        self.lbl_gust_val.config(text=f"{val:.1f} m/s")
        self.update_gust_calculations(val)

    def update_gust_calculations(self, v):
        # Rayleigh Fluid Drag: Fd = 0.5 * rho * Cd * A * v^2
        # rho = 1.225, Cd = 1.15, Area = 0.22 m^2
        f_drag = 0.5 * 1.225 * 1.15 * 0.22 * (v ** 2)
        # Required tilt angle: sin(theta) = F_drag / (m * g)
        mg = 1.50 * 9.81 # 14.715 N
        ratio = min(0.7071, f_drag / mg)
        theta_deg = math.degrees(math.asin(ratio))
        t_thrust = mg / max(0.5, math.cos(math.radians(theta_deg)))

        # Update real-time calculated labels
        self.lbl_calc_drag.config(text=f"Fluid Drag Force (Fd = ½ρCdAv²): {f_drag:.2f} N")
        self.lbl_calc_tilt.config(text=f"Req. Counter-Tilt: {theta_deg:.1f}° | Thrust: {t_thrust:.2f} N")
        self.btn_inject_gust.config(text=f"⚡ INJECT {v:.1f} m/s GUST (Exp. Force: {f_drag:.2f} N)")

    def do_trigger_gust(self):
        v = float(self.scale_gust.get())
        msg = Float32()
        msg.data = v
        self.ros_node.gust_pub.publish(msg)

    def toggle_dob(self):
        self.ros_node.dob_active = not self.ros_node.dob_active
        msg = Bool()
        msg.data = self.ros_node.dob_active
        self.ros_node.dob_pub.publish(msg)

        if self.ros_node.dob_active:
            self.btn_dob_toggle.config(text="DOB ACTIVE (Zero-Drift Wind Rejection)", bg='#238636')
        else:
            self.btn_dob_toggle.config(text="BASELINE PID (Disturbance Observer Off)", bg='#30363d')

    # -------------------------------------------------------------
    # Keyboard Handling & Debouncing
    # -------------------------------------------------------------
    def on_key_press(self, event):
        k = event.keysym.lower()
        if k in self._release_timers:
            self.root.after_cancel(self._release_timers[k])
            del self._release_timers[k]

        if k in ['t']:
            self.do_takeoff()
        elif k in ['l']:
            self.do_land()
        elif k in ['h', 'space']:
            self.do_hover()
        elif k in ['c']:
            self.do_clear_path()
        else:
            if self.patrol_active:
                self.do_abort_patrol(reason="PILOT KEY OVERRIDE")
            self.active_keys.add(k)
            self.update_key_button_highlight(k, is_pressed=True)

    def on_key_release(self, event):
        k = event.keysym.lower()
        if k in ['t', 'l', 'h', 'space', 'c']:
            return
        if k in self._release_timers:
            self.root.after_cancel(self._release_timers[k])
        self._release_timers[k] = self.root.after(70, lambda key=k: self._confirmed_key_release(key))

    def _confirmed_key_release(self, k):
        if k in self._release_timers:
            del self._release_timers[k]
        if k in self.active_keys:
            self.active_keys.remove(k)
            self.update_key_button_highlight(k, is_pressed=False)

    def update_key_button_highlight(self, key, is_pressed):
        active_bg = '#444c56'
        normal_bg = '#30363d'
        if key in ['w', 'up']:
            self.btn_fwd.config(bg=active_bg if is_pressed else normal_bg)
        elif key in ['s', 'down']:
            self.btn_back.config(bg=active_bg if is_pressed else normal_bg)
        elif key in ['a', 'left']:
            self.btn_left.config(bg=active_bg if is_pressed else normal_bg)
        elif key in ['d', 'right']:
            self.btn_right.config(bg=active_bg if is_pressed else normal_bg)
        elif key in ['i', 'e', 'prior']:
            self.btn_climb.config(bg=active_bg if is_pressed else normal_bg)
        elif key in ['k', 'q', 'next']:
            self.btn_descend.config(bg=active_bg if is_pressed else normal_bg)
        elif key in ['j']:
            self.btn_yaw_l.config(bg=active_bg if is_pressed else normal_bg)
        elif key in ['semicolon', 'bracketright']:
            self.btn_yaw_r.config(bg=active_bg if is_pressed else normal_bg)

    def reset_button_colors(self):
        for k in ['w', 's', 'a', 'd', 'i', 'k', 'j', 'semicolon']:
            self.update_key_button_highlight(k, False)

    def teleop_stream_loop(self):
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
            twist.linear.x = float(vx)
            twist.linear.y = float(vy)
            twist.linear.z = float(vz)
            twist.angular.z = float(vyaw)
            self.ros_node.cmd_vel_pub.publish(twist)
            self._is_streaming = True
        elif self._is_streaming:
            self.ros_node.cmd_vel_pub.publish(Twist())
            self._is_streaming = False

        self.root.after(35, self.teleop_stream_loop)

    def update_telemetry_loop(self):
        with self.ros_node.lock:
            p = list(self.ros_node.pos)
            v = list(self.ros_node.vel)
            y = self.ros_node.yaw
            st = self.ros_node.status
            fd = list(self.ros_node.est_dist)

        spd = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
        f_mag = math.sqrt(fd[0]**2 + fd[1]**2 + fd[2]**2)

        if st == "LANDED":
            self.lbl_status_banner.config(text="● SYSTEM STANDBY: ON GROUND SKIDS (Z = 0.12m)", fg='#d29922')
        elif st == "TAKEOFF":
            self.lbl_status_banner.config(text=f"▲ ASCENDING: CLIMBING TO HOVER ({p[2]:.2f}m)...", fg='#58a6ff')
        elif st == "HOVER":
            self.lbl_status_banner.config(text=f"■ POSITION HOLD: HOVERING AT {p[2]:.2f}m", fg='#3fb950')
        elif st == "TELEOP":
            self.lbl_status_banner.config(text=f"▶ FLIGHT ACTIVE: {spd:.2f} m/s (MANUAL PILOT)", fg='#3fb950')
        elif st == "TRAJECTORY":
            wp_num = self.patrol_wp_idx + 1 if self.patrol_active else 1
            tot = len(self.patrol_waypoints)
            self.lbl_status_banner.config(text=f"✈ AUTONOMOUS PATROL ACTIVE: WP {wp_num}/{tot} ({spd:.2f} m/s)", fg='#58a6ff')
        elif st == "LANDING":
            self.lbl_status_banner.config(text=f"▼ DESCENDING: CONTROLLED LANDING ({p[2]:.2f}m)...", fg='#f85149')
        else:
            self.lbl_status_banner.config(text=f"STATUS: {st}", fg='#58a6ff')

        self.lbl_alt.config(text=f"Altitude (Z): {p[2]:.2f} m")
        self.lbl_vel.config(text=f"Ground Speed: {spd:.2f} m/s")
        self.lbl_pos.config(text=f"Position [X, Y]: [{p[0]:.2f}, {p[1]:.2f}] m")
        self.lbl_yaw.config(text=f"Heading: {y:.1f} deg")
        self.lbl_drag.config(text=f"Estimated Disturbance Drag: {f_mag:.2f} N")

        self.root.after(50, self.update_telemetry_loop)

def main(args=None):
    rclpy.init(args=args)
    bridge = GcsRosBridge()

    t = threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True)
    t.start()

    root = tk.Tk()
    app = OfficialGcsDashboard(root, bridge)

    try:
        root.mainloop()
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
