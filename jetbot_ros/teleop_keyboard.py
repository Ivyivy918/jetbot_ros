#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import os

# 速度設定
JETBOT_MAX_LIN_VEL = 0.5
JETBOT_MAX_ANG_VEL = 1.0

msg = """
Control Your JetBot!
---------------------------
Moving around:
   W
A  S  D

W/S : increase/decrease linear velocity
A/D : increase/decrease angular velocity
Space : force stop
Q : quit

---------------------------
"""

def get_key():
    """取得鍵盤輸入"""
    if os.name == 'nt':
        return msvcrt.getch().decode()
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        self.get_logger().info('Teleop Keyboard initialized')
    
    def update_velocity(self, key):
        """根據按鍵更新速度"""
        if key == 'w' or key == 'W':
            self.linear_vel = JETBOT_MAX_LIN_VEL
            self.angular_vel = 0.0
        elif key == 's' or key == 'S':
            self.linear_vel = -JETBOT_MAX_LIN_VEL
            self.angular_vel = 0.0
        elif key == 'a' or key == 'A':
            self.linear_vel = 0.0
            self.angular_vel = JETBOT_MAX_ANG_VEL
        elif key == 'd' or key == 'D':
            self.linear_vel = 0.0
            self.angular_vel = -JETBOT_MAX_ANG_VEL
        elif key == ' ':
            self.linear_vel = 0.0
            self.angular_vel = 0.0
        
        self.publish_velocity()
    
    def publish_velocity(self):
        """發布速度指令"""
        twist = Twist()
        twist.linear.x = self.linear_vel
        twist.angular.z = self.angular_vel
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
            key = get_key()
            
            if key == 'q' or key == 'Q':
                break
            
            node.update_velocity(key)
            rclpy.spin_once(node, timeout_sec=0.1)
            
    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        # 停止機器人
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()