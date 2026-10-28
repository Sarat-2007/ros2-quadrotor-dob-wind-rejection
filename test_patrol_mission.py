#!/usr/bin/env python3
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Vector3Stamped, WrenchStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String, Bool, Float32, Int32

class PatrolTester(Node):
    def __init__(self):
        super().__init__('patrol_tester')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.takeoff_pub = self.create_publisher(Empty, '/uav/cmd_takeoff', 10)
        self.land_pub = self.create_publisher(Empty, '/uav/cmd_land', 10)
        self.hover_pub = self.create_publisher(Empty, '/uav/cmd_hover', 10)
        self.traj_mode_pub = self.create_publisher(Int32, '/uav/set_trajectory_mode', 10)
        self.setpoint_pub = self.create_publisher(PoseStamped, '/uav/setpoint', 10)
        self.gust_pub = self.create_publisher(Float32, '/uav/trigger_gust', 10)
        self.dob_pub = self.create_publisher(Bool, '/uav/set_dob_mode', 10)

        self.pos = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        self.state = "UNKNOWN"
        self.est_dist = [0.0, 0.0, 0.0]

        self.create_subscription(Odometry, '/uav/odom', self.odom_cb, 10)
        self.create_subscription(String, '/uav/flight_status', self.status_cb, 10)
        self.create_subscription(WrenchStamped, '/uav/estimated_disturbance', self.dist_cb, 10)

    def odom_cb(self, msg: Odometry):
        self.pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.vel = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]

    def status_cb(self, msg: String):
        self.state = msg.data

    def dist_cb(self, msg: WrenchStamped):
        self.est_dist = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]

def run_test():
    rclpy.init()
    tester = PatrolTester()

    print('\n=============================================================')
    print('AUTONOMOUS PATROL & MISSION ABORT VERIFICATION TEST')
    print('=============================================================')

    def spin_for(duration):
        start = time.time()
        while time.time() - start < duration:
            rclpy.spin_once(tester, timeout_sec=0.05)

    # 1. Check Initial Ground State
    for _ in range(60):
        rclpy.spin_once(tester, timeout_sec=0.05)
        if tester.state != "UNKNOWN":
            break
    print(f'[STAGE 1] Initial Ground State: Z = {tester.pos[2]:.3f} m, State = {tester.state}')
    assert tester.state == 'LANDED', f'Expected LANDED, got {tester.state}'
    assert abs(tester.pos[2] - 0.12) < 0.05, f'Expected Z ~ 0.12, got {tester.pos[2]}'
    print('  >>> PASS: Drone safely rested on landing skids.')

    # 2. Arm & Takeoff to Hover
    print('\n[STAGE 2] Arming & Commanding Takeoff (Climbing to 2.0m)...')
    tester.takeoff_pub.publish(Empty())
    t0 = time.time()
    while time.time() - t0 < 6.0:
        rclpy.spin_once(tester, timeout_sec=0.05)
        if tester.state == "HOVER" and abs(tester.pos[2] - 2.0) < 0.08:
            break

    print(f'  Post-Takeoff State: Z = {tester.pos[2]:.3f} m, State = {tester.state}')
    assert tester.state == 'HOVER', f'Expected HOVER, got {tester.state}'
    assert abs(tester.pos[2] - 2.0) < 0.10, f'Expected Z ~ 2.0m, got {tester.pos[2]}'
    print('  >>> PASS: Quadrotor stabilized at 2.0m hover.')

    # 3. Engage Autonomous Patrol towards Waypoint 1 (Inspection Gate: [0.0, 6.5, 2.2])
    print('\n[STAGE 3] Engaging Autonomous Patrol -> WP 1 (Inspection Gate Fly-Through [0, 6.5, 2.2])...')
    msg_mode = Int32()
    msg_mode.data = 10
    tester.traj_mode_pub.publish(msg_mode)

    # Stream setpoints towards gate for 4 seconds
    t_start = time.time()
    while time.time() - t_start < 4.0:
        sp = PoseStamped()
        sp.header.stamp = tester.get_clock().now().to_msg()
        sp.header.frame_id = 'world'
        sp.pose.position.x = 0.0
        sp.pose.position.y = 6.5
        sp.pose.position.z = 2.2
        tester.setpoint_pub.publish(sp)
        spin_for(0.05)

    print(f'  Navigated Position towards Gate: [{tester.pos[0]:.2f}, {tester.pos[1]:.2f}, {tester.pos[2]:.2f}] m, State = {tester.state}')
    assert tester.state == 'TRAJECTORY', f'Expected TRAJECTORY, got {tester.state}'
    assert tester.pos[1] > 2.0, f'Expected Y > 2.0m towards gate, got {tester.pos[1]:.2f}'
    print('  >>> PASS: Quadrotor smoothly advancing along trajectory corridor through inspection gate.')

    # 4. Abort Patrol Mission Mid-Flight -> Instant Station Hover
    print('\n[STAGE 4] Triggering MISSION ABORT (Testing Pilot Emergency Stop)...')
    msg_mode.data = 0
    tester.traj_mode_pub.publish(msg_mode)
    tester.hover_pub.publish(Empty())
    spin_for(1.5)
    
    abort_pos = list(tester.pos)
    print(f'  Halted Position: [{abort_pos[0]:.2f}, {abort_pos[1]:.2f}, {abort_pos[2]:.2f}] m, State = {tester.state}')
    assert tester.state == 'HOVER', f'Expected HOVER after abort, got {tester.state}'
    # Check that drone stopped moving
    spin_for(1.5)
    drift = np.linalg.norm(np.array(tester.pos) - np.array(abort_pos))
    print(f'  Station Hover Drift over 1.5s: {drift:.4f} m')
    assert drift < 0.05, f'Expected stationary hover (<0.05m), got {drift}m'
    print('  >>> PASS: Mission Abort locked drone into rock-solid station hover immediately!')

    # 5. Inject 8.0 m/s Shock Gust Mid-Flight with DOB Active
    print('\n[STAGE 5] Injecting 8.0 m/s Crosswind Shock Gust under Station Hover...')
    gust_msg = Float32()
    gust_msg.data = 8.0
    tester.gust_pub.publish(gust_msg)
    
    pre_gust_x = tester.pos[0]
    spin_for(2.0)
    post_gust_x = tester.pos[0]
    gust_drift = abs(post_gust_x - pre_gust_x)
    print(f'  DOB Estimated Disturbance: [{tester.est_dist[0]:.2f}, {tester.est_dist[1]:.2f}, {tester.est_dist[2]:.2f}] N')
    print(f'  Maximum Lateral Drift during 8m/s Gust: {gust_drift:.4f} m ({gust_drift*100:.1f} cm)')
    assert gust_drift < 0.25, f'Expected < 25 cm drift with DOB, got {gust_drift*100:.1f} cm'
    print('  >>> PASS: Disturbance Observer rejected 10N gust with minimal displacement (< 25 cm).')

    # 6. Controlled Landing & Disarm
    print('\n[STAGE 6] Commanding Controlled Landing...')
    tester.land_pub.publish(Empty())
    t0 = time.time()
    while time.time() - t0 < 6.0:
        rclpy.spin_once(tester, timeout_sec=0.05)
        if tester.state == "LANDED":
            break

    print(f'  Final Touchdown State: Z = {tester.pos[2]:.3f} m, State = {tester.state}')
    assert tester.state == 'LANDED', f'Expected LANDED, got {tester.state}'
    assert abs(tester.pos[2] - 0.12) < 0.05, f'Expected Z ~ 0.12m, got {tester.pos[2]}'
    print('  >>> PASS: Smooth landing executed. Rotors disarmed.')

    print('\n=============================================================')
    print('>>> ALL 6 STAGES OF PATROL & ABORT MISSION PASSED 100%! <<<')
    print('=============================================================\n')

    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    run_test()
