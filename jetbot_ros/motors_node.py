#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from Adafruit_MotorHAT import Adafruit_MotorHAT
import atexit
import traceback
import time
import math

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
        self.stop_timeout = 0.15  # 若超過此秒數未收到cmd_vel，自動停止

        # 關閉時停止馬達
        atexit.register(self.stop_motors)

        # 訂閱 cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Odometry 相關設定
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 位置追蹤
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # 當前速度（用於 odometry）
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0

        # 啟動定時檢查計時器
        self.last_cmd_time = time.time()
        self.timer = self.create_timer(0.1, self.check_timeout)

        # Odometry 更新計時器（50Hz）
        self.odom_timer = self.create_timer(0.02, self.update_odometry)

        self.get_logger().info("Motors node ready - waiting for /cmd_vel commands")

    def cmd_vel_callback(self, msg):
        """
        接收 /cmd_vel 並控制馬達
        """
        try:
            linear_x = msg.linear.x
            angular_z = msg.angular.z
            self.last_cmd_time = time.time()  # 更新最後收到訊息時間

            # 儲存當前速度用於 odometry 計算
            self.current_linear_vel = linear_x
            self.current_angular_vel = angular_z

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

            else:  # 沒輸入
                self.stop_motors()

        except Exception as e:
            self.get_logger().error(f"Error in cmd_vel_callback: {e}")
            self.get_logger().error(traceback.format_exc())

    def check_timeout(self):
        """定時檢查是否超過 timeout 沒有收到指令"""
        if time.time() - self.last_cmd_time > self.stop_timeout:
            self.stop_motors()

    def set_motor_speed(self, motor, speed, motor_name=""):
        """設定單一馬達速度"""
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

    def update_odometry(self):
        """更新並發布 odometry"""
        try:
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9

            # 計算位移（使用簡單的差分驅動模型）
            delta_x = self.current_linear_vel * math.cos(self.theta) * dt
            delta_y = self.current_linear_vel * math.sin(self.theta) * dt
            delta_theta = self.current_angular_vel * dt

            # 更新位置
            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta

            # 發布 TF 變換 (odom -> base_footprint)
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'

            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0

            # 轉換角度為四元數
            q = self.euler_to_quaternion(0, 0, self.theta)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(t)

            # 發布 Odometry 消息
            odom = Odometry()
            odom.header.stamp = current_time.to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_footprint'

            # 位置
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.x = q[0]
            odom.pose.pose.orientation.y = q[1]
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]

            # 速度
            odom.twist.twist.linear.x = self.current_linear_vel
            odom.twist.twist.angular.z = self.current_angular_vel

            self.odom_pub.publish(odom)

            self.last_time = current_time

        except Exception as e:
            self.get_logger().error(f"Error in update_odometry: {e}")

    def euler_to_quaternion(self, roll, pitch, yaw):
        """將歐拉角轉換為四元數"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = sr * cp * cy - cr * sp * sy  # x
        q[1] = cr * sp * cy + sr * cp * sy  # y
        q[2] = cr * cp * sy - sr * sp * cy  # z
        q[3] = cr * cp * cy + sr * sp * sy  # w

        return q

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