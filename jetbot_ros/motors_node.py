#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from Adafruit_MotorHAT import Adafruit_MotorHAT
import atexit
import traceback

class MotorsNode(Node):
    """
    Simple motor control node for keyboard control (WASD)
    """

    MOTOR_LEFT = 1
    MOTOR_RIGHT = 2

    def __init__(self):
        super().__init__('motors_node')

        # 初始化 MotorHAT
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

        # 參數設定
        self.max_speed = 200  # 最大PWM速度 (0~255)
        self.wheel_base = 0.175  # 輪距（m）

        # 關閉時停止馬達
        atexit.register(self.stop_motors)

        # 訂閱 cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info("Motors node ready - waiting for /cmd_vel commands")

    def cmd_vel_callback(self, msg):
        """
        接收 /cmd_vel 並控制馬達
        """
        try:
            linear_x = msg.linear.x
            angular_z = msg.angular.z

            self.get_logger().info(f"Received cmd_vel: linear={linear_x:.2f}, angular={angular_z:.2f}")

            # ======================
            # 控制邏輯
            # ======================
            if linear_x > 0.1:  # 前進 (W)
                self.get_logger().info("Forward (W)")
                self.set_motor_speed(self.left_motor, 0.6, "LEFT")
                self.set_motor_speed(self.right_motor, 0.6, "RIGHT")

            elif linear_x < -0.1:  # 後退 (S)
                self.get_logger().info("Backward (S)")
                self.set_motor_speed(self.left_motor, -0.6, "LEFT")
                self.set_motor_speed(self.right_motor, -0.6, "RIGHT")

            elif angular_z > 0.1:  # 左轉 (A) → 只讓右輪轉
                self.get_logger().info("Turn LEFT (A): right wheel runs")
                self.set_motor_speed(self.left_motor, 0.0, "LEFT")
                self.set_motor_speed(self.right_motor, 0.4, "RIGHT")

            elif angular_z < -0.1:  # 右轉 (D) → 只讓左輪轉
                self.get_logger().info("Turn RIGHT (D): left wheel runs")
                self.set_motor_speed(self.left_motor, 0.4, "LEFT")
                self.set_motor_speed(self.right_motor, 0.0, "RIGHT")

            else:  # 停止
                self.get_logger().info("Stop (no input)")
                self.set_motor_speed(self.left_motor, 0.0, "LEFT")
                self.set_motor_speed(self.right_motor, 0.0, "RIGHT")

        except Exception as e:
            self.get_logger().error(f"Error in cmd_vel_callback: {e}")
            self.get_logger().error(traceback.format_exc())

    def set_motor_speed(self, motor, speed, motor_name=""):
        """
        設定單一馬達速度
        speed 範圍：-1.0 ~ 1.0
        """
        try:
            speed = max(min(speed, 1.0), -1.0)
            pwm_speed = int(abs(speed) * self.max_speed)

            if speed > 0.05:
                motor.run(Adafruit_MotorHAT.FORWARD)
                motor.setSpeed(pwm_speed)
                self.get_logger().info(f"{motor_name}: FORWARD PWM={pwm_speed}")
            elif speed < -0.05:
                motor.run(Adafruit_MotorHAT.BACKWARD)
                motor.setSpeed(pwm_speed)
                self.get_logger().info(f"{motor_name}: BACKWARD PWM={pwm_speed}")
            else:
                motor.run(Adafruit_MotorHAT.RELEASE)
                self.get_logger().info(f"{motor_name}: STOP")

        except Exception as e:
            self.get_logger().error(f"Error setting {motor_name} motor: {e}")
            self.get_logger().error(traceback.format_exc())

    def stop_motors(self):
        """停止所有馬達"""
        try:
            self.left_motor.run(Adafruit_MotorHAT.RELEASE)
            self.right_motor.run(Adafruit_MotorHAT.RELEASE)
            self.get_logger().info("Motors stopped")
        except Exception as e:
            self.get_logger().error(f"Failed to stop motors: {e}")

    def destroy_node(self):
        """安全停止"""
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
