#!/usr/bin/env python3
"""
Wind Field Generator Node
Simulates dynamic real-time wind environments:
- Mean constant wind (speed + direction)
- Dynamic 1-cosine gust profiles
- Atmospheric turbulence (filtered stochastic noise)
"""

import math
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped, Vector3
from std_msgs.msg import Float32, String

class WindFieldNode(Node):
    def __init__(self):
        super().__init__('wind_field_node')

        # Parameters
        self.declare_parameter('base_wind_speed', 0.0)      # m/s
        self.declare_parameter('base_wind_direction_deg', 0.0)  # degrees (0=From West/Towards East +X, 90=+Y)
        self.declare_parameter('vertical_wind', 0.0)        # m/s (+Z up)
        self.declare_parameter('turbulence_intensity', 0.0) # 0.0 to 1.0 (std dev scale)
        self.declare_parameter('update_rate_hz', 50.0)

        # Gust state tracking
        self.gust_active = False
        self.gust_amplitude = 0.0
        self.gust_duration = 3.0    # seconds
        self.gust_start_time = 0.0
        self.gust_direction_deg = 0.0

        # Turbulence low-pass filter state
        self.turb_state = [0.0, 0.0, 0.0]
        self.turb_alpha = 0.85 # Filter smoothing

        # Publishers & Subscribers
        self.wind_pub = self.create_publisher(Vector3Stamped, '/uav/wind_velocity', 10)
        self.gust_sub = self.create_subscription(Float32, '/uav/trigger_gust', self.trigger_gust_cb, 10)
        self.param_sub = self.create_subscription(Vector3, '/uav/set_wind', self.set_wind_cb, 10)

        rate = self.get_parameter('update_rate_hz').get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        self.get_logger().info('Wind Field Generator Node initialized at %.1f Hz' % rate)

    def trigger_gust_cb(self, msg: Float32):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.gust_active = True
        self.gust_amplitude = msg.data if msg.data > 0.0 else 8.0
        self.gust_start_time = now_sec
        # Gust along current mean direction
        self.gust_direction_deg = self.get_parameter('base_wind_direction_deg').get_parameter_value().double_value
        self.get_logger().info(f'>>> GUST TRIGGERED! Amplitude: {self.gust_amplitude:.1f} m/s, Duration: {self.gust_duration:.1f}s')

    def set_wind_cb(self, msg: Vector3):
        # msg.x = speed, msg.y = direction_deg, msg.z = turbulence
        self.set_parameters([
            rclpy.parameter.Parameter('base_wind_speed', rclpy.Parameter.Type.DOUBLE, float(msg.x)),
            rclpy.parameter.Parameter('base_wind_direction_deg', rclpy.Parameter.Type.DOUBLE, float(msg.y)),
            rclpy.parameter.Parameter('turbulence_intensity', rclpy.Parameter.Type.DOUBLE, float(msg.z)),
        ])

    def timer_callback(self):
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9

        base_speed = self.get_parameter('base_wind_speed').get_parameter_value().double_value
        base_dir_deg = self.get_parameter('base_wind_direction_deg').get_parameter_value().double_value
        vert_speed = self.get_parameter('vertical_wind').get_parameter_value().double_value
        turb_intensity = self.get_parameter('turbulence_intensity').get_parameter_value().double_value

        rad = math.radians(base_dir_deg)
        vx_mean = base_speed * math.cos(rad)
        vy_mean = base_speed * math.sin(rad)
        vz_mean = vert_speed

        # Calculate Gust (1 - cosine profile)
        vx_gust, vy_gust, vz_gust = 0.0, 0.0, 0.0
        if self.gust_active:
            dt = now_sec - self.gust_start_time
            if 0.0 <= dt <= self.gust_duration:
                # Standard 1-cosine gust shape
                gust_factor = 0.5 * (1.0 - math.cos(2.0 * math.pi * dt / self.gust_duration))
                current_gust_mag = self.gust_amplitude * gust_factor
                g_rad = math.radians(self.gust_direction_deg)
                vx_gust = current_gust_mag * math.cos(g_rad)
                vy_gust = current_gust_mag * math.sin(g_rad)
            else:
                self.gust_active = False

        # Calculate Turbulence (Dryden-like low-pass filtered random walk)
        if turb_intensity > 0.001:
            raw_nx = random.gauss(0, turb_intensity * 2.0)
            raw_ny = random.gauss(0, turb_intensity * 2.0)
            raw_nz = random.gauss(0, turb_intensity * 1.0)
            self.turb_state[0] = self.turb_alpha * self.turb_state[0] + (1.0 - self.turb_alpha) * raw_nx
            self.turb_state[1] = self.turb_alpha * self.turb_state[1] + (1.0 - self.turb_alpha) * raw_ny
            self.turb_state[2] = self.turb_alpha * self.turb_state[2] + (1.0 - self.turb_alpha) * raw_nz
        else:
            self.turb_state = [0.0, 0.0, 0.0]

        # Total dynamic wind vector
        total_vx = vx_mean + vx_gust + self.turb_state[0]
        total_vy = vy_mean + vy_gust + self.turb_state[1]
        total_vz = vz_mean + vz_gust + self.turb_state[2]

        msg = Vector3Stamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'world'
        msg.vector.x = float(total_vx)
        msg.vector.y = float(total_vy)
        msg.vector.z = float(total_vz)

        self.wind_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WindFieldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
