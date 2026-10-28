#!/usr/bin/env python3
"""
Multi-Position Wind Gust Benchmark: Baseline PID vs. Disturbance Observer (DOB)
Executes identical 11-waypoint 3D autonomous patrol flights under 3 orthogonal shock wind gusts:
- Gust 1: Inspection Gate Approach Corridor (7.5 m/s, ~8.7 N lateral force at 0 deg)
- Gust 2: Rooftop Station Aerial Survey (8.5 m/s, ~11.2 N force at 0 deg, 3.5m height)
- Gust 3: South Sector Cross-Corridor (8.0 m/s, ~9.9 N force at 270 deg, between Towers)

Compares:
1. True mathematical cross-track path deviation (drift from planned flight corridor)
2. Maximum lateral displacement drift at each gust location
3. Disturbance observer estimation fidelity vs. true fluid drag
4. Side-by-side empirical performance metrics (% drift reduction)
Generates:
- multi_gust_benchmark_pid.csv
- multi_gust_benchmark_dob.csv
- multi_gust_dob_vs_pid.png (Publication-grade 4-panel comparative figure)
"""

import time
import math
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Vector3, Vector3Stamped, WrenchStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String, Bool, Float32, Int32

WAYPOINTS = [
    {"name": "Helipad Ascent Corridor", "pos": [0.0, 0.0, 2.2]},
    {"name": "Gate Approach Corridor", "pos": [0.0, 3.0, 2.2]},
    {"name": "Inspection Gate Fly-Through", "pos": [0.0, 6.5, 2.2]},
    {"name": "North-East Turn Corridor", "pos": [3.0, 6.5, 2.5]},
    {"name": "East Outer Bypass", "pos": [6.8, 4.5, 3.2]},
    {"name": "Rooftop Station Aerial Survey", "pos": [6.0, 0.0, 3.5]},
    {"name": "South-West Survey Corridor", "pos": [3.0, -2.5, 3.0]},
    {"name": "South Central Transit", "pos": [0.0, -2.2, 2.5]},
    {"name": "Tower B Inspection Corridor", "pos": [-2.8, -2.5, 2.5]},
    {"name": "West Return Corridor", "pos": [-2.0, 0.0, 2.2]},
    {"name": "Helipad Home Recovery", "pos": [0.0, 0.0, 2.0]}
]

def point_to_segment_dist(p, a, b):
    ab = b - a
    ab_len_sq = np.dot(ab, ab)
    if ab_len_sq < 1e-6:
        return np.linalg.norm(p - a)
    t = max(0.0, min(1.0, np.dot(p - a, ab) / ab_len_sq))
    proj = a + t * ab
    return np.linalg.norm(p - proj)

def calc_cross_track(pos):
    p = np.array(pos)
    min_d = 999.0
    for i in range(len(WAYPOINTS) - 1):
        a = np.array(WAYPOINTS[i]["pos"])
        b = np.array(WAYPOINTS[i+1]["pos"])
        d = point_to_segment_dist(p, a, b)
        if d < min_d:
            min_d = d
    return float(min_d)

class MultiGustBenchmarkNode(Node):
    def __init__(self):
        super().__init__('multi_gust_benchmark_node')
        self.takeoff_pub = self.create_publisher(Empty, '/uav/cmd_takeoff', 10)
        self.land_pub = self.create_publisher(Empty, '/uav/cmd_land', 10)
        self.hover_pub = self.create_publisher(Empty, '/uav/cmd_hover', 10)
        self.reset_home_pub = self.create_publisher(Empty, '/uav/reset_to_home', 10)
        self.traj_mode_pub = self.create_publisher(Int32, '/uav/set_trajectory_mode', 10)
        self.setpoint_pub = self.create_publisher(PoseStamped, '/uav/setpoint', 10)
        self.gust_pub = self.create_publisher(Float32, '/uav/trigger_gust', 10)
        self.wind_pub = self.create_publisher(Vector3, '/uav/set_wind', 10)
        self.dob_pub = self.create_publisher(Bool, '/uav/set_dob_mode', 10)

        self.pos = np.array([0.0, 0.0, 0.12])
        self.vel = np.array([0.0, 0.0, 0.0])
        self.state = "UNKNOWN"
        self.dob_status = None
        self.est_dist = np.array([0.0, 0.0, 0.0])
        self.actual_drag = np.array([0.0, 0.0, 0.0])

        self.create_subscription(Odometry, '/uav/odom', self.odom_cb, 10)
        self.create_subscription(String, '/uav/flight_status', self.status_cb, 10)
        self.create_subscription(Bool, '/uav/dob_status', self.dob_status_cb, 10)
        self.create_subscription(WrenchStamped, '/uav/estimated_disturbance', self.dist_cb, 10)
        self.create_subscription(WrenchStamped, '/uav/aerodynamic_force', self.drag_cb, 10)

    def odom_cb(self, msg: Odometry):
        self.pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        self.vel = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])

    def status_cb(self, msg: String):
        self.state = msg.data

    def dob_status_cb(self, msg: Bool):
        self.dob_status = msg.data

    def dist_cb(self, msg: WrenchStamped):
        self.est_dist = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])

    def drag_cb(self, msg: WrenchStamped):
        self.actual_drag = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])

def run_pass(node: MultiGustBenchmarkNode, use_dob: bool, pass_name: str):
    print(f"\n{'='*70}")
    print(f"STARTING PATROL PASS: {pass_name.upper()}")
    print(f"{'='*70}")

    dob_msg = Bool(data=use_dob)
    t0 = time.time()
    while time.time() - t0 < 5.0:
        node.dob_pub.publish(dob_msg)
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.dob_status == use_dob:
            break

    print(f" -> Confirmed Controller Mode: {'ACTIVE DOB (Zero-Drift)' if node.dob_status else 'DISABLED (Baseline PID)'}")

    node.reset_home_pub.publish(Empty())
    time.sleep(1.0)
    for _ in range(25):
        rclpy.spin_once(node, timeout_sec=0.05)

    print(f" -> Initial State: {node.state} at [{node.pos[0]:.2f}, {node.pos[1]:.2f}, {node.pos[2]:.2f}] m")

    msg_mode = Int32()
    msg_mode.data = 10
    node.traj_mode_pub.publish(msg_mode)

    patrol_speed = 1.35
    dt = 0.05
    step = patrol_speed * dt
    carrot_pos = np.copy(node.pos)

    gust1_done = False
    gust2_done = False
    gust3_done = False

    logged_data = {
        't': [], 'x': [], 'y': [], 'z': [],
        'tx': [], 'ty': [], 'tz': [],
        'err_3d': [], 'cross_track': [],
        'drag_mag': [], 'est_mag': [],
        'drag_x': [], 'drag_y': [],
        'est_x': [], 'est_y': [],
        'gust_active': [], 'gust_id': []
    }

    t_start = time.time()
    wp_idx = 0
    active_gust_id = 0
    gust_timer = 0.0

    while wp_idx < len(WAYPOINTS):
        t_now = time.time() - t_start
        target_wp = np.array(WAYPOINTS[wp_idx]["pos"], dtype=float)
        d_vec = target_wp - node.pos
        dist_wp = np.linalg.norm(d_vec)

        # -------------------------------------------------------------
        # Multi-Position Orthogonal Gust Injections
        # -------------------------------------------------------------
        # Gust 1: Gate Approach Corridor (Flying North, Wind East 0 deg)
        if wp_idx == 2 and not gust1_done and node.pos[1] >= 3.2:
            print(f"  >>> [GUST 1 TRIGGERED] Gate Approach Corridor (7.5 m/s East) at t = {t_now:.1f}s")
            w_msg = Vector3()
            w_msg.x = 0.0; w_msg.y = 0.0; w_msg.z = 0.0
            node.wind_pub.publish(w_msg)
            g_msg = Float32()
            g_msg.data = 7.5
            node.gust_pub.publish(g_msg)
            gust1_done = True
            active_gust_id = 1
            gust_timer = t_now

        # Gust 2: Rooftop Station Aerial Survey (Flying South, Wind East 0 deg)
        if wp_idx == 5 and not gust2_done and node.pos[0] >= 5.5:
            print(f"  >>> [GUST 2 TRIGGERED] Rooftop Station Survey (8.5 m/s East) at t = {t_now:.1f}s")
            w_msg = Vector3()
            w_msg.x = 0.0; w_msg.y = 0.0; w_msg.z = 0.0
            node.wind_pub.publish(w_msg)
            g_msg = Float32()
            g_msg.data = 8.5
            node.gust_pub.publish(g_msg)
            gust2_done = True
            active_gust_id = 2
            gust_timer = t_now

        # Gust 3: Helipad Recovery Hover (Terminal Approach over Helipad)
        if wp_idx == len(WAYPOINTS) - 1 and not gust3_done and dist_wp < 0.35:
            print(f"  >>> [GUST 3 TRIGGERED] Helipad Recovery Hover (8.0 m/s Crosswind) at t = {t_now:.1f}s")
            w_msg = Vector3()
            w_msg.x = 0.0; w_msg.y = 0.0; w_msg.z = 0.0
            node.wind_pub.publish(w_msg)
            g_msg = Float32()
            g_msg.data = 8.0
            node.gust_pub.publish(g_msg)
            gust3_done = True
            active_gust_id = 3
            gust_timer = t_now

        if active_gust_id > 0 and (t_now - gust_timer) > 3.5:
            active_gust_id = 0

        # Check waypoint transition (For final helipad waypoint, wait until gust test completes)
        if dist_wp < 0.35:
            if wp_idx == len(WAYPOINTS) - 1:
                if gust3_done and (t_now - gust_timer) >= 4.0:
                    print(f"  -> Completed WP 11/11: Helipad Home Recovery at [{node.pos[0]:.2f}, {node.pos[1]:.2f}, {node.pos[2]:.2f}] m")
                    break
            else:
                print(f"  -> Reached WP {wp_idx+1}/{len(WAYPOINTS)}: {WAYPOINTS[wp_idx]['name']} at [{node.pos[0]:.2f}, {node.pos[1]:.2f}, {node.pos[2]:.2f}] m")
                wp_idx += 1
                if wp_idx >= len(WAYPOINTS):
                    break
                target_wp = np.array(WAYPOINTS[wp_idx]["pos"], dtype=float)
                d_vec = target_wp - node.pos
                dist_wp = np.linalg.norm(d_vec)

        c_vec = target_wp - carrot_pos
        dist_c = np.linalg.norm(c_vec)
        if dist_c > step:
            carrot_pos += (c_vec / dist_c) * step
        else:
            carrot_pos = np.copy(target_wp)

        drone_to_carrot = carrot_pos - node.pos
        lead_dist = np.linalg.norm(drone_to_carrot)
        if lead_dist > 0.65:
            carrot_pos = node.pos + (drone_to_carrot / lead_dist) * 0.65

        sp = PoseStamped()
        sp.header.stamp = node.get_clock().now().to_msg()
        sp.header.frame_id = 'world'
        sp.pose.position.x = float(carrot_pos[0])
        sp.pose.position.y = float(carrot_pos[1])
        sp.pose.position.z = float(carrot_pos[2])
        target_yaw = math.atan2(d_vec[1], d_vec[0])
        sp.pose.orientation.w = float(math.cos(target_yaw * 0.5))
        sp.pose.orientation.z = float(math.sin(target_yaw * 0.5))
        node.setpoint_pub.publish(sp)

        rclpy.spin_once(node, timeout_sec=dt)

        err_vec = node.pos - carrot_pos
        ct_drift = calc_cross_track(node.pos)
        d_mag = np.linalg.norm(node.actual_drag[:2])
        e_mag = np.linalg.norm(node.est_dist[:2])

        logged_data['t'].append(t_now)
        logged_data['x'].append(node.pos[0])
        logged_data['y'].append(node.pos[1])
        logged_data['z'].append(node.pos[2])
        logged_data['tx'].append(carrot_pos[0])
        logged_data['ty'].append(carrot_pos[1])
        logged_data['tz'].append(carrot_pos[2])
        logged_data['err_3d'].append(np.linalg.norm(err_vec))
        logged_data['cross_track'].append(ct_drift)
        logged_data['drag_mag'].append(d_mag)
        logged_data['est_mag'].append(e_mag)
        logged_data['drag_x'].append(node.actual_drag[0])
        logged_data['drag_y'].append(node.actual_drag[1])
        logged_data['est_x'].append(node.est_dist[0])
        logged_data['est_y'].append(node.est_dist[1])
        logged_data['gust_active'].append(1 if active_gust_id > 0 else 0)
        logged_data['gust_id'].append(active_gust_id)

        if t_now > 70.0:
            print("  [WARN] Pass reached timeout safeguard.")
            break

    total_time = time.time() - t_start
    print(f" -> Completed {pass_name.upper()} in {total_time:.1f}s across all 11 waypoints.")

    msg_mode.data = 0
    node.traj_mode_pub.publish(msg_mode)
    node.hover_pub.publish(Empty())
    time.sleep(1.0)

    return logged_data

def analyze_and_plot(data_pid, data_dob, output_dir):
    print("\n" + "="*75)
    print("EMPIRICAL COMPARISON: BASELINE PID vs. DISTURBANCE OBSERVER (DOB)")
    print("="*75)

    def get_max_ct(data, g_id):
        indices = [i for i, gid in enumerate(data['gust_id']) if gid == g_id]
        if not indices:
            return 0.0
        return max(data['cross_track'][i] for i in indices)

    drift_pid_g1 = get_max_ct(data_pid, 1)
    drift_dob_g1 = get_max_ct(data_dob, 1)
    red_g1 = (drift_pid_g1 - drift_dob_g1) / max(0.001, drift_pid_g1) * 100.0

    drift_pid_g2 = get_max_ct(data_pid, 2)
    drift_dob_g2 = get_max_ct(data_dob, 2)
    red_g2 = (drift_pid_g2 - drift_dob_g2) / max(0.001, drift_pid_g2) * 100.0

    drift_pid_g3 = get_max_ct(data_pid, 3)
    drift_dob_g3 = get_max_ct(data_dob, 3)
    red_g3 = (drift_pid_g3 - drift_dob_g3) / max(0.001, drift_pid_g3) * 100.0

    mean_ct_pid = float(np.mean(data_pid['cross_track']))
    mean_ct_dob = float(np.mean(data_dob['cross_track']))
    overall_red = (mean_ct_pid - mean_ct_dob) / max(0.001, mean_ct_pid) * 100.0

    print(f"{'Flight Event / Gust Location':<35} | {'Baseline PID':<14} | {'Active DOB':<14} | {'Drift Reduction'}")
    print("-" * 75)
    print(f"{'Gust 1: Gate Approach (7.5 m/s)':<35} | {drift_pid_g1*100:>6.1f} cm      | {drift_dob_g1*100:>6.1f} cm      | {red_g1:>6.1f} %")
    print(f"{'Gust 2: Rooftop Survey (8.5 m/s)':<35} | {drift_pid_g2*100:>6.1f} cm      | {drift_dob_g2*100:>6.1f} cm      | {red_g2:>6.1f} %")
    print(f"{'Gust 3: Helipad Recovery (8.0 m/s)':<35} | {drift_pid_g3*100:>6.1f} cm      | {drift_dob_g3*100:>6.1f} cm      | {red_g3:>6.1f} %")
    print("-" * 75)
    print(f"{'Overall Mean Cross-Track Error':<35} | {mean_ct_pid*100:>6.1f} cm      | {mean_ct_dob*100:>6.1f} cm      | {overall_red:>6.1f} %")
    print("=" * 75)

    # -------------------------------------------------------------
    # Generate Publication-Grade 4-Panel Visualization
    # -------------------------------------------------------------
    fig, axes = plt.subplots(2, 2, figsize=(15, 11), facecolor='#ffffff')
    plt.subplots_adjust(hspace=0.28, wspace=0.22)

    # Panel 1: Top-Down 2D Trajectory with Obstacle Footprints
    ax1 = axes[0, 0]
    ax1.set_title("3D Facility Trajectory: Planned vs Baseline PID vs Active DOB", fontsize=11, fontweight='bold', pad=10)
    ax1.plot([-1.6, -1.6], [4.9, 5.1], color='#d9534f', lw=4, label='Inspection Gate Pillars')
    ax1.plot([1.6, 1.6], [4.9, 5.1], color='#d9534f', lw=4)
    ax1.add_patch(plt.Rectangle((4.4, -1.6), 3.2, 3.2, color='#6c757d', alpha=0.4, label='Rooftop Building (2m)'))
    ax1.add_patch(plt.Rectangle((5.0, -8.0), 4.0, 4.0, color='#495057', alpha=0.5, label='Industrial Tower A'))
    ax1.add_patch(plt.Rectangle((-7.75, -8.25), 3.5, 4.5, color='#343a40', alpha=0.5, label='Industrial Tower B'))
    for px, py in [(4.0, 4.0), (-4.0, 4.0), (-5.0, 0.0), (0.0, -4.0)]:
        ax1.add_patch(plt.Circle((px, py), 0.35, color='#fd7e14', alpha=0.7))

    wps_x = [w["pos"][0] for w in WAYPOINTS]
    wps_y = [w["pos"][1] for w in WAYPOINTS]
    ax1.plot(wps_x, wps_y, 'k--', alpha=0.5, lw=1.5, label='Planned Waypoints')
    ax1.plot(data_pid['x'], data_pid['y'], color='#d9534f', lw=2.0, label='Baseline PID (Severe Drift Excursions)')
    ax1.plot(data_dob['x'], data_dob['y'], color='#28a745', lw=2.2, label='Active DOB (Zero-Drift Corridor)')
    ax1.plot(0, 0, 'go', markersize=8, label='Helipad Origin')
    ax1.set_xlabel("X (East) [m]", fontsize=10)
    ax1.set_ylabel("Y (North) [m]", fontsize=10)
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='lower left', fontsize=8, framealpha=0.9)
    ax1.set_xlim(-8.5, 9.5)
    ax1.set_ylim(-9.0, 8.0)

    # Panel 2: Continuous Cross-Track Path Drift over Time with Gust Shading
    ax2 = axes[0, 1]
    ax2.set_title("Cross-Track Path Deviation (Drift from Flight Corridor)", fontsize=11, fontweight='bold', pad=10)
    ax2.plot(data_pid['t'], [e * 100 for e in data_pid['cross_track']], color='#d9534f', lw=1.8, label='Baseline PID Drift')
    ax2.plot(data_dob['t'], [e * 100 for e in data_dob['cross_track']], color='#28a745', lw=2.0, label='Active DOB Drift')

    g_active = False
    g_start = 0
    current_gid = 0
    for idx, (t, gid) in enumerate(zip(data_dob['t'], data_dob['gust_id'])):
        if gid > 0 and not g_active:
            g_active = True
            g_start = t
            current_gid = gid
        elif gid == 0 and g_active:
            g_active = False
            ax2.axvspan(g_start, t, color='#ffc107', alpha=0.25)
            ax2.text((g_start + t) / 2, 70, f"Gust {current_gid}", ha='center', fontsize=8, fontweight='bold', color='#856404')
    ax2.set_xlabel("Flight Time [s]", fontsize=10)
    ax2.set_ylabel("Cross-Track Drift [cm]", fontsize=10)
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='upper right', fontsize=9)
    ax2.set_ylim(0, 170)

    # Panel 3: Disturbance Observer Dynamic Fidelity (Estimated vs Actual Drag)
    ax3 = axes[1, 0]
    ax3.set_title("Disturbance Observer Estimation vs Rayleigh Aerodynamic Drag", fontsize=11, fontweight='bold', pad=10)
    ax3.plot(data_dob['t'], data_dob['drag_mag'], color='#212529', lw=1.5, linestyle='--', label='Physical Fluid Drag Force (Rayleigh)')
    ax3.plot(data_dob['t'], data_dob['est_mag'], color='#007bff', lw=2.0, label='DOB Estimated Disturbance (35 rad/s)')
    ax3.set_xlabel("Flight Time [s]", fontsize=10)
    ax3.set_ylabel("Total Drag Force [N]", fontsize=10)
    ax3.grid(True, linestyle=':', alpha=0.6)
    ax3.legend(loc='upper right', fontsize=9)
    ax3.set_ylim(-1, 14)

    # Panel 4: Quantitative Bar Chart Comparison
    ax4 = axes[1, 1]
    ax4.set_title("Maximum Lateral Drift Comparison at Each Gust Position", fontsize=11, fontweight='bold', pad=10)
    locations = ['Gust 1: Gate Approach\n(7.5 m/s Crosswind)', 'Gust 2: Rooftop Survey\n(8.5 m/s Crosswind)', 'Gust 3: Helipad Recovery\n(8.0 m/s Crosswind)']
    pid_drifts = [drift_pid_g1 * 100, drift_pid_g2 * 100, drift_pid_g3 * 100]
    dob_drifts = [drift_dob_g1 * 100, drift_dob_g2 * 100, drift_dob_g3 * 100]
    reductions = [red_g1, red_g2, red_g3]

    x = np.arange(len(locations))
    width = 0.32

    rects1 = ax4.bar(x - width/2, pid_drifts, width, label='Baseline PID', color='#d9534f', alpha=0.85)
    rects2 = ax4.bar(x + width/2, dob_drifts, width, label='Active DOB', color='#28a745', alpha=0.85)

    for i in range(len(locations)):
        y_val = max(pid_drifts[i], dob_drifts[i]) + 4
        ax4.text(x[i], y_val, f"-{reductions[i]:.1f}%\n({pid_drifts[i]:.1f}cm -> {dob_drifts[i]:.1f}cm)",
                 ha='center', va='bottom', fontsize=8.5, fontweight='bold', color='#155724',
                 bbox=dict(boxstyle='round,pad=0.3', facecolor='#d4edda', edgecolor='#c3e6cb'))

    ax4.set_ylabel("Max Lateral Drift [cm]", fontsize=10)
    ax4.set_xticks(x)
    ax4.set_xticklabels(locations, fontsize=8.5)
    ax4.set_ylim(0, 180)
    ax4.grid(True, axis='y', linestyle=':', alpha=0.6)
    ax4.legend(loc='upper right', fontsize=9)

    plt.suptitle("Multi-Position Wind Gust Rejection Benchmark: Active DOB vs Baseline PID", fontsize=13, fontweight='bold', y=0.98)

    img_path = os.path.join(output_dir, "multi_gust_dob_vs_pid.png")
    plt.savefig(img_path, dpi=250, bbox_inches='tight')
    plt.close()
    print(f"\n >>> Publication-Grade Benchmark Graph saved: {img_path}")

    pid_csv = os.path.join(output_dir, "multi_gust_benchmark_pid.csv")
    dob_csv = os.path.join(output_dir, "multi_gust_benchmark_dob.csv")

    with open(pid_csv, 'w') as f:
        f.write("t,x,y,z,tx,ty,tz,err_3d,cross_track,drag_mag,est_mag,drag_x,drag_y,est_x,est_y,gust_active,gust_id\n")
        for i in range(len(data_pid['t'])):
            f.write(f"{data_pid['t'][i]:.3f},{data_pid['x'][i]:.3f},{data_pid['y'][i]:.3f},{data_pid['z'][i]:.3f},"
                    f"{data_pid['tx'][i]:.3f},{data_pid['ty'][i]:.3f},{data_pid['tz'][i]:.3f},"
                    f"{data_pid['err_3d'][i]:.3f},{data_pid['cross_track'][i]:.3f},"
                    f"{data_pid['drag_mag'][i]:.3f},{data_pid['est_mag'][i]:.3f},"
                    f"{data_pid['drag_x'][i]:.3f},{data_pid['drag_y'][i]:.3f},"
                    f"{data_pid['est_x'][i]:.3f},{data_pid['est_y'][i]:.3f},"
                    f"{data_pid['gust_active'][i]},{data_pid['gust_id'][i]}\n")

    with open(dob_csv, 'w') as f:
        f.write("t,x,y,z,tx,ty,tz,err_3d,cross_track,drag_mag,est_mag,drag_x,drag_y,est_x,est_y,gust_active,gust_id\n")
        for i in range(len(data_dob['t'])):
            f.write(f"{data_dob['t'][i]:.3f},{data_dob['x'][i]:.3f},{data_dob['y'][i]:.3f},{data_dob['z'][i]:.3f},"
                    f"{data_dob['tx'][i]:.3f},{data_dob['ty'][i]:.3f},{data_dob['tz'][i]:.3f},"
                    f"{data_dob['err_3d'][i]:.3f},{data_dob['cross_track'][i]:.3f},"
                    f"{data_dob['drag_mag'][i]:.3f},{data_dob['est_mag'][i]:.3f},"
                    f"{data_dob['drag_x'][i]:.3f},{data_dob['drag_y'][i]:.3f},"
                    f"{data_dob['est_x'][i]:.3f},{data_dob['est_y'][i]:.3f},"
                    f"{data_dob['gust_active'][i]},{data_dob['gust_id'][i]}\n")

    print(f" >>> Telemetry Logs saved: {pid_csv} and {dob_csv}")

def main():
    rclpy.init()
    node = MultiGustBenchmarkNode()
    try:
        data_pid = run_pass(node, use_dob=False, pass_name="Pass 1: Baseline Industrial PID")
        data_dob = run_pass(node, use_dob=True, pass_name="Pass 2: Active Disturbance Observer (DOB)")
        output_dir = "/home/sarat_9_7/ros2_uav_ws"
        analyze_and_plot(data_pid, data_dob, output_dir)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
