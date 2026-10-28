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
import tkinter as tk
from tkinter import ttk
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, WrenchStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String, Bool, Float32

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
        self.root.geometry("560x780")
        self.root.configure(bg='#1a1d24')
        self.root.resizable(False, False)

        # Flight velocity parameters
        self.teleop_speed = 1.2
        self.teleop_climb_speed = 0.8
        self.teleop_yaw_speed = 0.8

        self.active_keys = set()
        self._release_timers = {}
        self._is_streaming = False

        self.setup_ui()
        self.bind_keyboard()
        self.update_telemetry_loop()
        self.teleop_stream_loop()

        self.root.focus_force()

    def bind_keyboard(self):
        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.bind('<KeyRelease>', self.on_key_release)

    def bind_dpad_hold(self, widget, key_code):
        widget.bind('<ButtonPress-1>', lambda e: self.on_gui_btn_down(key_code))
        widget.bind('<ButtonRelease-1>', lambda e: self.on_gui_btn_up(key_code))

    def on_gui_btn_down(self, key_code):
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

        # 3. Manual Flight Stick (D-Pad)
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

        c2 = tk.Frame(ctrl_box, bg='#242933')
        c2.pack(fill=tk.X, pady=(6, 2))
        tk.Label(c2, text="Simulated Wind Velocity:", bg='#242933', fg='#8b949e', font=('Helvetica', 9)).pack(side=tk.LEFT)
        self.lbl_wind_val = tk.Label(c2, text="0.0 m/s", bg='#242933', fg='#58a6ff', font=('Helvetica', 9, 'bold'))
        self.lbl_wind_val.pack(side=tk.RIGHT)

        c3 = tk.Frame(ctrl_box, bg='#242933')
        c3.pack(fill=tk.X, pady=2)
        self.scale_wind = ttk.Scale(c3, from_=0.0, to=12.0, value=0.0, command=self.on_wind_slide)
        self.scale_wind.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 6))

        tk.Button(c3, text="Inject Gust (Slider)", bg='#30363d', fg='#e6edf3', font=('Helvetica', 8, 'bold'),
                  relief='flat', command=self.do_trigger_gust).pack(side=tk.RIGHT)

        # Quick Gust Presets
        c4 = tk.Frame(ctrl_box, bg='#242933')
        c4.pack(fill=tk.X, pady=(3, 2))
        tk.Label(c4, text="Quick Shock Gusts:", bg='#242933', fg='#8b949e', font=('Helvetica', 8)).pack(side=tk.LEFT)
        for g_val in [4.0, 8.0, 10.0]:
            tk.Button(c4, text=f"{int(g_val)} m/s", bg='#30363d', fg='#e6edf3', font=('Helvetica', 8),
                      relief='flat', command=lambda v=g_val: self.do_trigger_custom_gust(v)).pack(side=tk.LEFT, padx=3)

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
        self.ros_node.land_pub.publish(Empty())

    def do_hover(self):
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
        self.ros_node.reset_home_pub.publish(Empty())

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

    def do_trigger_gust(self):
        slider_val = self.scale_wind.get()
        gust_mag = float(slider_val) if slider_val >= 1.0 else 8.0
        msg = Float32()
        msg.data = gust_mag
        self.ros_node.gust_pub.publish(msg)

    def do_trigger_custom_gust(self, val):
        msg = Float32()
        msg.data = float(val)
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
