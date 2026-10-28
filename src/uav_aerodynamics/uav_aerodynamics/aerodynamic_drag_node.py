#!/usr/bin/env python3
"""
Aerodynamic Drag Calculation Node
Calculates real-time 3D aerodynamic forces acting on the drone:
  V_rel = V_drone - V_wind
  F_drag = -0.5 * rho * Cd * A * |V_rel| * V_rel
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped, Vector3, WrenchStamped
from nav_msgs.msg import Odometry

class AerodynamicDragNode(Node):
    def __init__(self):
        super().__init__('aerodynamic_drag_node')

        # Parameters
        self.declare_parameter('air_density', 1.225)      # kg/m^3
        self.declare_parameter('drag_coefficient', 1.15)  # Bluff-body quadrotor Cd
        self.declare_parameter('frontal_area', 0.22)       # Effective area in m^2
        self.declare_parameter('mass', 1.5)               # Total drone mass in kg

        # State storage
        self.wind_vel = [0.0, 0.0, 0.0]
        self.drone_vel = [0.0, 0.0, 0.0]
        self.drone_pos = [0.0, 0.0, 0.0]

        # Publishers
        self.force_pub = self.create_publisher(WrenchStamped, '/uav/aerodynamic_force', 10)
        self.rel_vel_pub = self.create_publisher(Vector3Stamped, '/uav/relative_velocity', 10)

        # Subscribers
        self.wind_sub = self.create_subscription(Vector3Stamped, '/uav/wind_velocity', self.wind_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/uav/odom', self.odom_callback, 10)
        self.aero_param_sub = self.create_subscription(Vector3, '/uav/set_aero_params', self.set_aero_params_cb, 10)

        # 50 Hz calculation loop
        self.timer = self.create_timer(0.02, self.compute_drag_force)
        self.get_logger().info('Aerodynamic Drag Model Node initialized (50 Hz)')

    def wind_callback(self, msg: Vector3Stamped):
        self.wind_vel = [msg.vector.x, msg.vector.y, msg.vector.z]

    def odom_callback(self, msg: Odometry):
        self.drone_vel = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ]
        self.drone_pos = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]

    def set_aero_params_cb(self, msg: Vector3):
        # msg.x = Cd, msg.y = Area, msg.z = rho
        if msg.x > 0:
            self.set_parameters([rclpy.parameter.Parameter('drag_coefficient', rclpy.Parameter.Type.DOUBLE, float(msg.x))])
        if msg.y > 0:
            self.set_parameters([rclpy.parameter.Parameter('frontal_area', rclpy.Parameter.Type.DOUBLE, float(msg.y))])
        if msg.z > 0:
            self.set_parameters([rclpy.parameter.Parameter('air_density', rclpy.Parameter.Type.DOUBLE, float(msg.z))])

    def compute_drag_force(self):
        rho = self.get_parameter('air_density').get_parameter_value().double_value
        cd = self.get_parameter('drag_coefficient').get_parameter_value().double_value
        area = self.get_parameter('frontal_area').get_parameter_value().double_value

        # Relative air velocity: V_rel = V_drone - V_wind
        v_rel_x = self.drone_vel[0] - self.wind_vel[0]
        v_rel_y = self.drone_vel[1] - self.wind_vel[1]
        v_rel_z = self.drone_vel[2] - self.wind_vel[2]

        v_rel_mag = math.sqrt(v_rel_x**2 + v_rel_y**2 + v_rel_z**2)

        # Aerodynamic Drag: F_d = -0.5 * rho * Cd * A * |V_rel| * V_rel
        coeff = 0.5 * rho * cd * area * v_rel_mag
        f_drag_x = -coeff * v_rel_x
        f_drag_y = -coeff * v_rel_y
        f_drag_z = -coeff * v_rel_z

        now = self.get_clock().now()

        # Publish Wrench
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = now.to_msg()
        wrench_msg.header.frame_id = 'world'
        wrench_msg.wrench.force.x = float(f_drag_x)
        wrench_msg.wrench.force.y = float(f_drag_y)
        wrench_msg.wrench.force.z = float(f_drag_z)
        self.force_pub.publish(wrench_msg)

        # Publish Relative Velocity
        rel_vel_msg = Vector3Stamped()
        rel_vel_msg.header.stamp = now.to_msg()
        rel_vel_msg.header.frame_id = 'world'
        rel_vel_msg.vector.x = float(v_rel_x)
        rel_vel_msg.vector.y = float(v_rel_y)
        rel_vel_msg.vector.z = float(v_rel_z)
        self.rel_vel_pub.publish(rel_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AerodynamicDragNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
