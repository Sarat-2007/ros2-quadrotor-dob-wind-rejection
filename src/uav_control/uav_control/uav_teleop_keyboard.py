#!/usr/bin/env python3
"""
Interactive Terminal Keyboard Teleoperation for UAV Simulation
Allows intuitive manual flying:
- 't' : Takeoff to 2.0m hover
- 'l' : Land on ground
- 'h' : Hover / Hold position
- 'w'/'s' : Forward / Backward (+X / -X)
- 'a'/'d' : Left / Right Strafe (+Y / -Y)
- 'i'/'k' or 'e'/'q' : Climb / Descend (+Z / -Z)
- 'j'/';' : Yaw rotation Left / Right
- 'c' : Clear trajectory path
- Space : Stop / Brake to Hover
"""

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String

BANNER = """
======================================================================
       REAL-TIME UAV KEYBOARD FLIGHT & TELEOPERATION CONSOLE
======================================================================
  [T]       : 🚀 TAKEOFF (Lifts off ground skids -> 2.0m Hover)
  [L]       : 🛬 LAND (Smooth controlled descent -> Touchdown)
  [H]       : 🎯 HOVER (Instant position hold at current location)

  MANUAL FLIGHT KEYS:
         [W] Forward (+X)
    [A] Left (+Y)     [D] Right (-Y)
         [S] Backward (-X)

    [I] / [E] : Climb Ascend (+Z)
    [K] / [Q] : Descend (-Z)
    [J] : Yaw Rotate Left     [;] : Yaw Rotate Right

  [C]       : 🧹 Clear Visual Path Trail
  [SPACE]   : ⏸ Brake / Stop at current position
  [Ctrl+C]  : Exit Teleoperation
======================================================================
"""

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class UavTeleopNode(Node):
    def __init__(self):
        super().__init__('uav_teleop_keyboard')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.takeoff_pub = self.create_publisher(Empty, '/uav/cmd_takeoff', 10)
        self.land_pub = self.create_publisher(Empty, '/uav/cmd_land', 10)
        self.hover_pub = self.create_publisher(Empty, '/uav/cmd_hover', 10)
        self.clear_pub = self.create_publisher(Empty, '/uav/clear_path', 10)
        self.status_sub = self.create_subscription(String, '/uav/flight_status', self.status_cb, 10)

        self.current_status = "LANDED"
        self.speed_xy = 1.2 # m/s
        self.speed_z = 0.8  # m/s
        self.speed_yaw = 0.8 # rad/s

    def status_cb(self, msg: String):
        self.current_status = msg.data

def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = UavTeleopNode()

    print(BANNER)
    print("Teleop node connected to ROS 2. Ready for keyboard input!\n")

    twist = Twist()
    last_key_time = 0

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            key = get_key(settings)

            if key:
                key_l = key.lower()

                if key_l == '\x03': # Ctrl+C
                    break

                elif key_l == 't':
                    node.takeoff_pub.publish(Empty())
                    print(">>> [T] TAKEOFF Command Sent: Quadrotor launching to 2.0m Hover!")

                elif key_l == 'l':
                    node.land_pub.publish(Empty())
                    print(">>> [L] LAND Command Sent: Quadrotor descending for touchdown!")

                elif key_l == 'h' or key == ' ':
                    node.hover_pub.publish(Empty())
                    twist = Twist()
                    node.cmd_pub.publish(twist)
                    print(">>> [H] HOVER Command Sent: Holding station.")

                elif key_l == 'c':
                    node.clear_pub.publish(Empty())
                    print(">>> [C] Visual Path Cleared in RViz!")

                elif key_l == 'w':
                    twist = Twist()
                    twist.linear.x = node.speed_xy
                    node.cmd_pub.publish(twist)
                    print(">>> Flying FORWARD (+X)")

                elif key_l == 's':
                    twist = Twist()
                    twist.linear.x = -node.speed_xy
                    node.cmd_pub.publish(twist)
                    print(">>> Flying BACKWARD (-X)")

                elif key_l == 'a':
                    twist = Twist()
                    twist.linear.y = node.speed_xy
                    node.cmd_pub.publish(twist)
                    print(">>> Strafing LEFT (+Y)")

                elif key_l == 'd':
                    twist = Twist()
                    twist.linear.y = -node.speed_xy
                    node.cmd_pub.publish(twist)
                    print(">>> Strafing RIGHT (-Y)")

                elif key_l in ['i', 'e']:
                    twist = Twist()
                    twist.linear.z = node.speed_z
                    node.cmd_pub.publish(twist)
                    print(">>> CLIMBING Upward (+Z)")

                elif key_l in ['k', 'q']:
                    twist = Twist()
                    twist.linear.z = -node.speed_z
                    node.cmd_pub.publish(twist)
                    print(">>> DESCENDING Downward (-Z)")

                elif key_l == 'j':
                    twist = Twist()
                    twist.angular.z = node.speed_yaw
                    node.cmd_pub.publish(twist)
                    print(">>> Rotating YAW LEFT (CCW)")

                elif key_l in [';', 'l_bracket']:
                    twist = Twist()
                    twist.angular.z = -node.speed_yaw
                    node.cmd_pub.publish(twist)
                    print(">>> Rotating YAW RIGHT (CW)")

            else:
                # Key released: publish zero velocity to automatically hold position
                pass

    except Exception as e:
        print("Teleop error:", e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
