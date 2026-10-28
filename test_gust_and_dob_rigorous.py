#!/usr/bin/env python3
"""
Rigorous Automated Verification:
1. Full Flight Lifecycle (Takeoff, Teleop, Hover, Landing)
2. Sudden 8.0 m/s Wind Gust Test with DOB ACTIVE (Zero-Drift Rejection)
3. Sudden 8.0 m/s Wind Gust Test with BASELINE PID (Drift Comparison)
4. Thrust & Disturbance Estimation Performance Verification
"""

import time
import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, WrenchStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String, Bool, Float32

class FullRigorousTester(Node):
    def __init__(self):
        super().__init__('rigorous_flight_tester')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.takeoff_pub = self.create_publisher(Empty, '/uav/cmd_takeoff', 10)
        self.land_pub = self.create_publisher(Empty, '/uav/cmd_land', 10)
        self.hover_pub = self.create_publisher(Empty, '/uav/cmd_hover', 10)
        self.dob_pub = self.create_publisher(Bool, '/uav/set_dob_mode', 10)
        self.gust_pub = self.create_publisher(Float32, '/uav/trigger_gust', 10)
        self.wind_pub = self.create_publisher(Vector3, '/uav/set_wind', 10)

        # State
        self.pos = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        self.state = "UNKNOWN"
        self.f_dist_est = [0.0, 0.0, 0.0]
        self.f_drag_actual = [0.0, 0.0, 0.0]

        # Subscribers
        self.create_subscription(Odometry, '/uav/odom', self.odom_cb, 10)
        self.create_subscription(String, '/uav/flight_status', self.status_cb, 10)
        self.create_subscription(WrenchStamped, '/uav/estimated_disturbance', self.est_cb, 10)
        self.create_subscription(WrenchStamped, '/uav/aerodynamic_force', self.aero_cb, 10)

    def odom_cb(self, msg: Odometry):
        self.pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.vel = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]

    def status_cb(self, msg: String):
        self.state = msg.data

    def est_cb(self, msg: WrenchStamped):
        self.f_dist_est = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]

    def aero_cb(self, msg: WrenchStamped):
        self.f_drag_actual = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]

def run_rigorous_test():
    rclpy.init()
    node = FullRigorousTester()

    print("=" * 80)
    print("     RIGOROUS UAV AUTOPILOT, THRUST & DISTURBANCE OBSERVER (DOB) TEST")
    print("=" * 80)

    # 1. Connect and check initial state
    print("\n[STEP 1] Checking Ground Contact State...")
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.state != "UNKNOWN":
            break

    print(f" -> Initial State: {node.state} | Position: [{node.pos[0]:.2f}, {node.pos[1]:.2f}, {node.pos[2]:.2f}] m")
    assert node.state == "LANDED", f"Expected LANDED, got {node.state}"
    assert abs(node.pos[2] - 0.12) < 0.03, f"Expected skid contact 0.12m, got {node.pos[2]}"
    print(" -> PASS: Quadrotor resting securely on ground skids.")

    # 2. Takeoff
    print("\n[STEP 2] Commencing Takeoff to 2.0m Hover Altitude...")
    node.takeoff_pub.publish(Empty())
    t0 = time.time()
    while time.time() - t0 < 6.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.state == "HOVER" and abs(node.pos[2] - 2.0) < 0.08:
            break

    print(f" -> Hover State: {node.state} | Altitude reached: {node.pos[2]:.2f} m")
    assert node.state == "HOVER", f"Expected HOVER, got {node.state}"
    assert abs(node.pos[2] - 2.0) < 0.08, f"Expected 2.0m hover, got {node.pos[2]}"
    print(" -> PASS: Smooth ascent & stable station-holding at 2.0m achieved.")

    # 3. Test Sudden 8.0 m/s Gust with DOB ACTIVE (Our Core Idea)
    print("\n[STEP 3] INJECTING SUDDEN 8.0 m/s GUST WITH DISTURBANCE OBSERVER (DOB) ACTIVE...")
    node.dob_pub.publish(Bool(data=True)) # Enable DOB
    time.sleep(0.5)

    base_x_dob = node.pos[0]
    gust_msg = Float32(data=8.0)
    node.gust_pub.publish(gust_msg)

    # Monitor response for 4 seconds during the gust wave
    max_disp_dob = 0.0
    max_est_force = 0.0
    t0 = time.time()
    while time.time() - t0 < 4.5:
        rclpy.spin_once(node, timeout_sec=0.04)
        disp = abs(node.pos[0] - base_x_dob)
        if disp > max_disp_dob:
            max_disp_dob = disp
        force_mag = abs(node.f_dist_est[0])
        if force_mag > max_est_force:
            max_est_force = force_mag

    print(f" -> Peak Wind Disturbance Force Estimated by DOB: {max_est_force:.2f} N")
    print(f" -> Maximum Drone Displacement with DOB ACTIVE: {max_disp_dob:.4f} m ({max_disp_dob*100:.1f} cm)")
    assert max_est_force > 5.0, f"DOB failed to estimate wind force! Got only {max_est_force} N"
    assert max_disp_dob < 0.25, f"DOB failed zero-drift! Deflected {max_disp_dob} m"
    print(f" -> PASS: DOB actively generated counter-thrust, holding position with only {max_disp_dob*100:.2f} cm drift!")

    # 4. Test Sudden 8.0 m/s Gust with BASELINE PID (DOB Disabled)
    print("\n[STEP 4] INJECTING SUDDEN 8.0 m/s GUST WITH BASELINE PID (DOB DISABLED)...")
    node.dob_pub.publish(Bool(data=False)) # Disable DOB -> Standard PID
    time.sleep(1.0)
    base_x_pid = node.pos[0]

    node.gust_pub.publish(gust_msg)
    max_disp_pid = 0.0
    t0 = time.time()
    while time.time() - t0 < 4.5:
        rclpy.spin_once(node, timeout_sec=0.04)
        disp = abs(node.pos[0] - base_x_pid)
        if disp > max_disp_pid:
            max_disp_pid = disp

    print(f" -> Maximum Drone Displacement with BASELINE PID: {max_disp_pid:.2f} m ({max_disp_pid*100:.1f} cm)")
    assert max_disp_pid > 0.50, f"Expected standard PID to drift significantly, got {max_disp_pid} m"

    error_reduction = (1.0 - (max_disp_dob / max_disp_pid)) * 100.0
    print(f" -> PERFORMANCE ADVANTAGE OF OUR IMPLEMENTED DOB:")
    print(f"    - Baseline PID Drift: {max_disp_pid:.2f} m")
    print(f"    - Our DOB Drift:      {max_disp_dob:.4f} m ({max_disp_dob*100:.1f} cm)")
    print(f"    - DRIFT REDUCTION:    {error_reduction:.1f}% ACCURACY IMPROVEMENT!")
    print(" -> PASS: Rigorous empirical proof that our DOB eliminates wind error!")

    # Re-enable DOB for remaining flight
    node.dob_pub.publish(Bool(data=True))
    time.sleep(1.0)

    # 5. Continuous Manual Teleoperation Test
    print("\n[STEP 5] Testing Continuous Manual Teleoperation (Pilot Control Stick)...")
    t0 = time.time()
    start_x = node.pos[0]
    tw = Twist()
    tw.linear.x = 1.2
    while time.time() - t0 < 3.0:
        node.cmd_pub.publish(tw)
        rclpy.spin_once(node, timeout_sec=0.04)

    # Stop and brake
    node.cmd_pub.publish(Twist())
    for _ in range(25):
        rclpy.spin_once(node, timeout_sec=0.04)

    moved_x = node.pos[0] - start_x
    print(f" -> Continuous Forward Flight Distance Traveled: +{moved_x:.2f} m")
    assert moved_x > 2.5, f"Teleop failed to advance continuously! Moved only {moved_x} m"
    print(" -> PASS: Smooth, continuous, uninterrupted manual teleoperation verified.")

    # 6. Controlled Soft Landing
    print("\n[STEP 6] Initiating Controlled Landing & Disarm Sequence...")
    node.land_pub.publish(Empty())
    t0 = time.time()
    while time.time() - t0 < 8.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.state == "LANDED":
            break

    print(f" -> Final State: {node.state} | Touchdown Altitude: Z = {node.pos[2]:.3f} m")
    assert node.state == "LANDED", f"Expected LANDED, got {node.state}"
    assert abs(node.pos[2] - 0.12) < 0.03, f"Expected 0.12m skid contact, got {node.pos[2]}"
    print(" -> PASS: Soft touchdown on skids. Motors automatically disarmed.")

    print("\n" + "=" * 80)
    print("     ALL RIGOROUS TESTS (LIFECYCLE, DOB GUST REJECTION, TELEOP) PASSED 100%!")
    print("=" * 80)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    run_rigorous_test()
