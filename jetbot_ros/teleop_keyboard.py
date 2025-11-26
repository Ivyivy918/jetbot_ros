#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import os
import select

JETBOT_MAX_LIN_VEL = 0.5
JETBOT_MAX_ANG_VEL = 1.0

msg = """
Control Your JetBot!
---------------------------
   W
A  S  D

W/S : forward/backward
A/D : turn left/right
Q : quit
---------------------------
"""

def get_key(timeout=0.1):
    if os.name == 'nt':
        import msvcrt
        if msvcrt.kbhit():
            return msvcrt.getch().decode()
        return ''
    
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Teleop Keyboard initialized')
    
    def process_key(self, key):
        linear_vel = 0.0
        angular_vel = 0.0
        
        if key == 'w' or key == 'W':
            linear_vel = JETBOT_MAX_LIN_VEL
        elif key == 's' or key == 'S':
            linear_vel = -JETBOT_MAX_LIN_VEL
        elif key == 'a' or key == 'A':
            angular_vel = JETBOT_MAX_ANG_VEL
        elif key == 'd' or key == 'D':
            angular_vel = -JETBOT_MAX_ANG_VEL
        
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    global settings
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init(args=args)
    node = TeleopKeyboard()
    print(msg)
    
    try:
        while rclpy.ok():
            key = get_key(0.1)
            if key == 'q' or key == 'Q':
                break
            node.process_key(key)
            rclpy.spin_once(node, timeout_sec=0.01)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()