#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from Adafruit_MotorHAT import Adafruit_MotorHAT
import atexit
import traceback

class MotorsNode(Node):
    """
    Simple motor control node for keyboard control
    """
    
    MOTOR_LEFT = 1
    MOTOR_RIGHT = 2
    
    def __init__(self):
        super().__init__('motors_node')
        
        # Initialize MotorHAT
        try:
            self.get_logger().info("Initializing MotorHAT on i2c_bus=7...")
            self.driver = Adafruit_MotorHAT(i2c_bus=7)
            self.left_motor = self.driver.getMotor(self.MOTOR_LEFT)
            self.right_motor = self.driver.getMotor(self.MOTOR_RIGHT)
            self.get_logger().info("MotorHAT initialized successfully")
        except Exception as e:
            self.get_logger().error(f"MotorHAT initialization failed: {e}")
            self.get_logger().error(traceback.format_exc())
            raise
        
        # Motor parameters
        self.max_speed = 200  # Maximum PWM speed (0-255)，降低一點測試
        self.wheel_base = 0.175  # Wheel separation in meters (17.5cm)
        
        # 註冊關閉時停止馬達
        atexit.register(self.stop_motors)
        
        # Subscribe to keyboard control (cmd_vel)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.get_logger().info("Motors node ready - waiting for /cmd_vel commands")
    
    def cmd_vel_callback(self, msg):
        """
        Receive velocity commands from keyboard
        """
        try:
            # 除錯訊息
            self.get_logger().info(f"Received cmd_vel: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}")
            
            # Calculate differential drive speeds
            linear_x = msg.linear.x
            angular_z = msg.angular.z
            
            # Differential drive kinematics
            left_speed = linear_x - angular_z * (self.wheel_base / 2)
            right_speed = linear_x + angular_z * (self.wheel_base / 2)
            
            self.get_logger().info(f"Calculated motor speeds: left={left_speed:.2f}, right={right_speed:.2f}")
            
            # Set motor speeds
            self.get_logger().info("Setting left motor...")
            self.set_motor_speed(self.left_motor, left_speed, "LEFT")
            
            self.get_logger().info("Setting right motor...")
            self.set_motor_speed(self.right_motor, right_speed, "RIGHT")
            
            self.get_logger().info("Motors set successfully")
            
        except Exception as e:
            self.get_logger().error(f"Error in cmd_vel_callback: {e}")
            self.get_logger().error(traceback.format_exc())
    
    def set_motor_speed(self, motor, speed, motor_name=""):
        """
        Set motor speed and direction
        speed: -1.0 to 1.0 (normalized)
        """
        try:
            self.get_logger().info(f"{motor_name}: Starting set_motor_speed with speed={speed:.2f}")
            
            # Limit speed range
            speed = max(min(speed, 1.0), -1.0)
            
            # Convert to PWM (0-255)
            pwm_speed = int(abs(speed) * self.max_speed)
            
            self.get_logger().info(f"{motor_name}: PWM={pwm_speed}, speed={speed:.2f}")
            
            # Set direction
            if speed > 0.05:  # Forward (with deadzone)
                self.get_logger().info(f"{motor_name}: Running FORWARD")
                motor.run(Adafruit_MotorHAT.FORWARD)
                motor.setSpeed(pwm_speed)
                self.get_logger().info(f"{motor_name}: FORWARD at PWM {pwm_speed} - SUCCESS")
                
            elif speed < -0.05:  # Backward (with deadzone)
                self.get_logger().info(f"{motor_name}: Running BACKWARD")
                motor.run(Adafruit_MotorHAT.BACKWARD)
                motor.setSpeed(pwm_speed)
                self.get_logger().info(f"{motor_name}: BACKWARD at PWM {pwm_speed} - SUCCESS")
                
            else:  # Stop
                self.get_logger().info(f"{motor_name}: RELEASE (STOP)")
                motor.run(Adafruit_MotorHAT.RELEASE)
                self.get_logger().info(f"{motor_name}: STOP - SUCCESS")
                
        except Exception as e:
            self.get_logger().error(f"Error setting {motor_name} motor: {e}")
            self.get_logger().error(traceback.format_exc())
    
    def stop_motors(self):
        """Stop all motors"""
        try:
            self.left_motor.run(Adafruit_MotorHAT.RELEASE)
            self.right_motor.run(Adafruit_MotorHAT.RELEASE)
            self.get_logger().info("Motors stopped")
        except Exception as e:
            self.get_logger().error(f"Failed to stop motors: {e}")
    
    def destroy_node(self):
        """Safely stop motors when node is destroyed"""
        self.get_logger().info("Shutting down motors node...")
        self.stop_motors()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotorsNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
        traceback.print_exc()
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()