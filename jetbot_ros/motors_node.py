#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from Adafruit_MotorHAT import Adafruit_MotorHAT

class MotorNode(Node):
    """
    Motor control node
    """
    
    MOTOR_LEFT = 1
    MOTOR_RIGHT = 2
    
    def __init__(self):
        super().__init__('motor_node')
        
        # Initialize MotorHAT
        try:
            self.driver = Adafruit_MotorHAT(i2c_bus=7)
            self.left_motor = self.driver.getMotor(self.MOTOR_LEFT)
            self.right_motor = self.driver.getMotor(self.MOTOR_RIGHT)
            self.get_logger().info("MotorHAT initialized successfully")
        except Exception as e:
            self.get_logger().error(f"MotorHAT initialization failed: {e}")
            raise
        
        # Motor parameters
        self.rotation_speed = 120  # PWM speed (0-255)
        self.is_rotating = False
        self.current_distance = -1.0  # Store current distance
        
        # Subscribe to obstacle detection from camera node
        self.obstacle_sub = self.create_subscription(
            Bool,
            'obstacle_detected',  # Topic published by camera node
            self.obstacle_callback,
            10
        )
        
        # Subscribe to distance messages
        self.distance_sub = self.create_subscription(
            Float32,
            'obstacle_distance',  # Distance topic from camera node
            self.distance_callback,
            10
        )
        
        # Publish motor status
        self.motor_status_pub = self.create_publisher(Bool, 'motor_status', 10)
        
        # Timer for status publishing
        self.create_timer(1.0, self.publish_status)
        
        # Start rotation on startup
        self.start_rotation()
        
        print("Motor control node started")
        print("Waiting for camera obstacle detection messages...")
    
    def obstacle_callback(self, msg):
        """
        Receive obstacle detection messages
        True = obstacle detected, stop motor
        False = no obstacle, start rotation
        """
        obstacle_detected = msg.data
        
        if obstacle_detected and self.is_rotating:
            # Obstacle detected and currently rotating -> stop
            self.stop_rotation()
            
        elif not obstacle_detected and not self.is_rotating:
            # No obstacle and currently stopped -> start rotation
            self.start_rotation()
    
    def distance_callback(self, msg):
        """
        Receive distance messages
        """
        self.current_distance = msg.data
    
    def start_rotation(self):
        """Start both motors rotating clockwise"""
        if not self.is_rotating:
            try:
                # Set both motors to same direction (clockwise)
                # Assuming FORWARD is clockwise direction
                self.left_motor.setSpeed(self.rotation_speed)
                self.right_motor.setSpeed(self.rotation_speed)
                
                self.left_motor.run(Adafruit_MotorHAT.FORWARD)
                self.right_motor.run(Adafruit_MotorHAT.FORWARD)
                
                self.is_rotating = True
                print("Move forward")
                
            except Exception as e:
                self.get_logger().error(f"Failed to start motors: {e}")
    
    def stop_rotation(self):
        """Stop both motors"""
        if self.is_rotating:
            try:
                self.left_motor.run(Adafruit_MotorHAT.RELEASE)
                self.right_motor.run(Adafruit_MotorHAT.RELEASE)
                
                self.is_rotating = False
                
                # Print stop message with distance if available
                if self.current_distance > 0:
                    distance_cm = self.current_distance * 100  # Convert to cm
                    print(f"Stop - Distance: {distance_cm:.1f}cm")
                else:
                    print("Stop")
                
            except Exception as e:
                self.get_logger().error(f"Failed to stop motors: {e}")
    
    def publish_status(self):
        """Publish motor status"""
        status_msg = Bool()
        status_msg.data = self.is_rotating
        self.motor_status_pub.publish(status_msg)
    
    def destroy_node(self):
        """Safely stop motors when node is destroyed"""
        print("Safely stopping motors...")
        try:
            self.left_motor.run(Adafruit_MotorHAT.RELEASE)
            self.right_motor.run(Adafruit_MotorHAT.RELEASE)
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = MotorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()