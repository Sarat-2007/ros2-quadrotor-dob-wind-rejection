#!/usr/bin/env python3
"""
Continuous Trajectory Generator Node with Analytical Velocity & Feedforward Acceleration
Features:
- Analytical C2 Continuous Trajectory Generation
- Mode 0: Stationary Hover at target coordinates
- Mode 1: Directed Velocity Flight with smooth S-curve ramp
- Mode 2: Dynamic Circular Orbit with smooth spiral ingress & centripetal feedforward
- Mode 3: Continuous Spline Waypoint Circuit (smooth C2 traversal without discrete jumps)
- Simultaneous publishing of Setpoint Position, Velocity, and Feedforward Acceleration
- Instant Return to Home & Reset
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from std_msgs.msg import Int32, Float32, Empty
from nav_msgs.msg import Path
from scipy.interpolate import splprep, splev

class TrajectoryGeneratorNode(Node):
    def __init__(self):
        super().__init__('trajectory_generator_node')

        # Parameters & State
        self.mode = 0  # 0=Hover, 1=Directed Velocity, 2=Circle, 3=Smooth Spline Waypoints
        self.hover_height = 2.0
        self.flight_speed = 1.5       # m/s
        self.flight_heading_deg = 0.0  # deg (0=East, 90=North)

        # Dynamic setpoint coordinates
        self.current_setpoint = np.array([0.0, 0.0, self.hover_height])
        self.current_vel = np.array([0.0, 0.0, 0.0])
        self.current_acc = np.array([0.0, 0.0, 0.0])

        # Mode 1: S-curve ramp
        self.mode1_speed_curr = 0.0
        self.mode1_ramp_time = 1.2
        self.mode1_timer = 0.0

        # Mode 2: Circle parameters
        self.circle_radius = 2.5
        self.circle_theta = 0.0
        self.circle_ingress_r = 0.0
        self.circle_ingress_time = 2.0
        self.circle_timer = 0.0

        # Mode 3: Closed periodic spline waypoint circuit (0,0) -> (3,0) -> (3,3) -> (0,3) -> (0,0)
        wps_x = [0.0, 3.0, 3.0, 0.0, 0.0]
        wps_y = [0.0, 0.0, 3.0, 3.0, 0.0]
        self.spline_tck, _ = splprep([wps_x, wps_y], s=0, per=True, k=3)
        self.wp_circuit_time = 16.0 # seconds per full circuit
        self.wp_timer = 0.0

        # Publishers
        self.setpoint_pub = self.create_publisher(PoseStamped, '/uav/setpoint', 10)
        self.setpoint_vel_pub = self.create_publisher(Vector3Stamped, '/uav/setpoint_velocity', 10)
        self.setpoint_acc_pub = self.create_publisher(Vector3Stamped, '/uav/setpoint_accel', 10)
        self.ref_path_pub = self.create_publisher(Path, '/uav/reference_path', 10)

        # Subscribers
        self.create_subscription(Int32, '/uav/set_trajectory_mode', self.mode_cb, 10)
        self.create_subscription(Float32, '/uav/set_flight_speed', self.speed_cb, 10)
        self.create_subscription(Float32, '/uav/set_flight_heading', self.heading_cb, 10)
        self.create_subscription(Empty, '/uav/reset_to_home', self.reset_home_cb, 10)

        self.dt = 0.02 # 50 Hz
        self.timer = self.create_timer(self.dt, self.timer_cb)
        self.publish_reference_path()
        self.get_logger().info('Trajectory Generator active with Velocity Feedforward & Analytical Splines')

    def mode_cb(self, msg: Int32):
        old_mode = self.mode
        self.mode = msg.data
        if self.mode not in [1, 2, 3]:
            self.current_vel = np.array([0.0, 0.0, 0.0])
            self.current_acc = np.array([0.0, 0.0, 0.0])
        elif self.mode == 1:
            self.mode1_timer = 0.0
            self.mode1_speed_curr = 0.0
        elif self.mode == 2:
            self.circle_timer = 0.0
            self.circle_ingress_r = 0.0
            self.circle_theta = 0.0
        elif self.mode == 3:
            self.wp_timer = 0.0

        self.publish_reference_path()
        self.get_logger().info(f'Trajectory Mode switched from {old_mode} to {self.mode}')

    def speed_cb(self, msg: Float32):
        self.flight_speed = max(0.1, float(msg.data))
        self.get_logger().info(f'Flight Speed set to: {self.flight_speed:.2f} m/s')

    def heading_cb(self, msg: Float32):
        self.flight_heading_deg = float(msg.data) % 360.0
        self.publish_reference_path()

    def reset_home_cb(self, msg: Empty):
        self.current_setpoint = np.array([0.0, 0.0, self.hover_height])
        self.current_vel = np.array([0.0, 0.0, 0.0])
        self.current_acc = np.array([0.0, 0.0, 0.0])
        self.mode = 0
        self.circle_theta = 0.0
        self.circle_timer = 0.0
        self.wp_timer = 0.0
        self.publish_reference_path()
        self.get_logger().info('Setpoint reset to Origin Hover (0, 0, 2m)')

    def publish_reference_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'world'

        if self.mode == 0:
            p = PoseStamped()
            p.header.frame_id = 'world'
            p.pose.position.x = float(self.current_setpoint[0])
            p.pose.position.y = float(self.current_setpoint[1])
            p.pose.position.z = float(self.current_setpoint[2])
            path.poses.append(p)
        elif self.mode == 1:
            rad = math.radians(self.flight_heading_deg)
            for dist in np.linspace(0.0, 8.0, 17):
                p = PoseStamped()
                p.header.frame_id = 'world'
                p.pose.position.x = float(self.current_setpoint[0] + dist * math.cos(rad))
                p.pose.position.y = float(self.current_setpoint[1] + dist * math.sin(rad))
                p.pose.position.z = self.hover_height
                path.poses.append(p)
        elif self.mode == 2:
            r = self.circle_radius
            for deg in range(0, 365, 5):
                rad = math.radians(deg)
                p = PoseStamped()
                p.header.frame_id = 'world'
                p.pose.position.x = r * math.cos(rad)
                p.pose.position.y = r * math.sin(rad)
                p.pose.position.z = self.hover_height
                path.poses.append(p)
        elif self.mode == 3:
            # High-resolution spline curve preview
            u_vals = np.linspace(0.0, 1.0, 100)
            pts = splev(u_vals, self.spline_tck, der=0)
            for x, y in zip(pts[0], pts[1]):
                p = PoseStamped()
                p.header.frame_id = 'world'
                p.pose.position.x = float(x)
                p.pose.position.y = float(y)
                p.pose.position.z = self.hover_height
                path.poses.append(p)

        self.ref_path_pub.publish(path)

    def timer_cb(self):
        if self.mode not in [1, 2, 3]:
            return
        now = self.get_clock().now()

        elif self.mode == 1:
            # Directed Velocity Flight with smooth S-curve ramp
            self.mode1_timer += self.dt
            rad = math.radians(self.flight_heading_deg)
            u_dir = np.array([math.cos(rad), math.sin(rad), 0.0])

            if self.mode1_timer < self.mode1_ramp_time:
                frac = self.mode1_timer / self.mode1_ramp_time
                s = 0.5 * (1.0 - math.cos(math.pi * frac))
                s_dot = 0.5 * (math.pi / self.mode1_ramp_time) * math.sin(math.pi * frac)
                self.current_vel = u_dir * (self.flight_speed * s)
                self.current_acc = u_dir * (self.flight_speed * s_dot)
            else:
                self.current_vel = u_dir * self.flight_speed
                self.current_acc = np.array([0.0, 0.0, 0.0])

            self.current_setpoint += self.current_vel * self.dt
            self.current_setpoint[2] = self.hover_height

        elif self.mode == 2:
            # Dynamic Circular Orbit with smooth spiral ingress & centripetal feedforward
            self.circle_timer += self.dt
            R = self.circle_radius
            omg = self.flight_speed / R

            if self.circle_timer < self.circle_ingress_time:
                frac = self.circle_timer / self.circle_ingress_time
                s = 0.5 * (1.0 - math.cos(math.pi * frac))
                s_dot = 0.5 * (math.pi / self.circle_ingress_time) * math.sin(math.pi * frac)
                s_ddot = 0.5 * ((math.pi / self.circle_ingress_time)**2) * math.cos(math.pi * frac)
                r = R * s
                r_dot = R * s_dot
                r_ddot = R * s_ddot
                theta = omg * self.circle_timer

                self.current_setpoint = np.array([r * math.cos(theta), r * math.sin(theta), self.hover_height])
                self.current_vel = np.array([
                    r_dot * math.cos(theta) - r * omg * math.sin(theta),
                    r_dot * math.sin(theta) + r * omg * math.cos(theta),
                    0.0
                ])
                self.current_acc = np.array([
                    r_ddot * math.cos(theta) - 2 * r_dot * omg * math.sin(theta) - r * (omg**2) * math.cos(theta),
                    r_ddot * math.sin(theta) + 2 * r_dot * omg * math.cos(theta) - r * (omg**2) * math.sin(theta),
                    0.0
                ])
            else:
                theta = omg * self.circle_timer
                self.current_setpoint = np.array([R * math.cos(theta), R * math.sin(theta), self.hover_height])
                self.current_vel = np.array([-self.flight_speed * math.sin(theta), self.flight_speed * math.cos(theta), 0.0])
                self.current_acc = np.array([-(omg**2) * self.current_setpoint[0], -(omg**2) * self.current_setpoint[1], 0.0])

        elif self.mode == 3:
            # Continuous Spline Waypoint Circuit
            self.wp_timer += self.dt
            T = self.wp_circuit_time
            u_val = (self.wp_timer % T) / T

            pos = splev(u_val, self.spline_tck, der=0)
            vel = splev(u_val, self.spline_tck, der=1)
            acc = splev(u_val, self.spline_tck, der=2)

            self.current_setpoint = np.array([float(pos[0]), float(pos[1]), self.hover_height])
            self.current_vel = np.array([float(vel[0]) / T, float(vel[1]) / T, 0.0])
            self.current_acc = np.array([float(acc[0]) / (T**2), float(acc[1]) / (T**2), 0.0])

        # 1. Publish Setpoint Position
        p_msg = PoseStamped()
        p_msg.header.stamp = now.to_msg()
        p_msg.header.frame_id = 'world'
        p_msg.pose.position.x = float(self.current_setpoint[0])
        p_msg.pose.position.y = float(self.current_setpoint[1])
        p_msg.pose.position.z = float(self.current_setpoint[2])
        p_msg.pose.orientation.w = 1.0
        self.setpoint_pub.publish(p_msg)

        # 2. Publish Setpoint Velocity
        v_msg = Vector3Stamped()
        v_msg.header.stamp = now.to_msg()
        v_msg.header.frame_id = 'world'
        v_msg.vector.x = float(self.current_vel[0])
        v_msg.vector.y = float(self.current_vel[1])
        v_msg.vector.z = float(self.current_vel[2])
        self.setpoint_vel_pub.publish(v_msg)

        # 3. Publish Setpoint Acceleration Feedforward
        a_msg = Vector3Stamped()
        a_msg.header.stamp = now.to_msg()
        a_msg.header.frame_id = 'world'
        a_msg.vector.x = float(self.current_acc[0])
        a_msg.vector.y = float(self.current_acc[1])
        a_msg.vector.z = float(self.current_acc[2])
        self.setpoint_acc_pub.publish(a_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
