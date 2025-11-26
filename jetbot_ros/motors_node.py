#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from Adafruit_MotorHAT import Adafruit_MotorHAT
import atexit
import traceback
import time
import math

class MotorsNode(Node):
    MOTOR_LEFT = 1
    MOTOR_RIGHT = 2

    def __init__(self):
        super().__init__('motors_node')

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

        self.max_speed = 200
        self.wheel_base = 0.175
        self.stop_timeout = 0.15

        atexit.register(self.stop_motors)

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0

        self.last_cmd_time = time.time()
        self.timer = self.create_timer(0.1, self.check_timeout)
        self.odom_timer = self.create_timer(0.02, self.update_odometry)

        self.get_logger().info("Motors node ready")

    def cmd_vel_callback(self, msg):
        try:
            linear_x = msg.linear.x
            angular_z = msg.angular.z
            self.last_cmd_time = time.time()

            self.current_linear_vel = linear_x
            self.current_angular_vel = angular_z

            if linear_x > 0.1:
                self.set_motor_speed(self.left_motor, 0.8, "LEFT")
                self.set_motor_speed(self.right_motor, 0.8, "RIGHT")
            elif linear_x < -0.1:
                self.set_motor_speed(self.left_motor, -0.8, "LEFT")
                self.set_motor_speed(self.right_motor, -0.8, "RIGHT")
            elif angular_z > 0.1:
                self.set_motor_speed(self.left_motor, 0.0, "LEFT")
                self.set_motor_speed(self.right_motor, 1, "RIGHT")
            elif angular_z < -0.1:
                self.set_motor_speed(self.left_motor, 1, "LEFT")
                self.set_motor_speed(self.right_motor, 0.0, "RIGHT")
            else:
                self.stop_motors()

        except Exception as e:
            self.get_logger().error(f"Error in cmd_vel_callback: {e}")

    def check_timeout(self):
        if time.time() - self.last_cmd_time > self.stop_timeout:
            self.current_linear_vel = 0.0
            self.current_angular_vel = 0.0
            self.stop_motors()

    def set_motor_speed(self, motor, speed, motor_name=""):
        try:
            speed = max(min(speed, 1.0), -1.0)
            pwm_speed = int(abs(speed) * self.max_speed)

            if speed > 0.05:
                motor.run(Adafruit_MotorHAT.FORWARD)
                motor.setSpeed(pwm_speed)
            elif speed < -0.05:
                motor.run(Adafruit_MotorHAT.BACKWARD)
                motor.setSpeed(pwm_speed)
            else:
                motor.run(Adafruit_MotorHAT.RELEASE)

        except Exception as e:
            self.get_logger().error(f"Error setting {motor_name} motor: {e}")

    def stop_motors(self):
        try:
            self.left_motor.run(Adafruit_MotorHAT.RELEASE)
            self.right_motor.run(Adafruit_MotorHAT.RELEASE)
            self.current_linear_vel = 0.0
            self.current_angular_vel = 0.0
        except Exception as e:
            self.get_logger().error(f"Failed to stop motors: {e}")

    def update_odometry(self):
        try:
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9

            delta_x = self.current_linear_vel * math.cos(self.theta) * dt
            delta_y = self.current_linear_vel * math.sin(self.theta) * dt
            delta_theta = self.current_angular_vel * dt

            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta

            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0

            q = self.euler_to_quaternion(0, 0, self.theta)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(t)

            odom = Odometry()
            odom.header.stamp = current_time.to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_footprint'
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.x = q[0]
            odom.pose.pose.orientation.y = q[1]
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]
            odom.twist.twist.linear.x = self.current_linear_vel
            odom.twist.twist.angular.z = self.current_angular_vel

            self.odom_pub.publish(odom)
            self.last_time = current_time

        except Exception as e:
            self.get_logger().error(f"Error in update_odometry: {e}")

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        return [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy
        ]

    def destroy_node(self):
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