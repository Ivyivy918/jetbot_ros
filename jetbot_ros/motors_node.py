#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from Adafruit_MotorHAT import Adafruit_MotorHAT

class MotorNode(Node):
    """
    馬達控制節點
    """
    
    MOTOR_LEFT = 1
    MOTOR_RIGHT = 2
    
    def __init__(self):
        super().__init__('motor_node')
        
        # 初始化 MotorHAT
        try:
            self.driver = Adafruit_MotorHAT(i2c_bus=7)
            self.left_motor = self.driver.getMotor(self.MOTOR_LEFT)
            self.right_motor = self.driver.getMotor(self.MOTOR_RIGHT)
            self.get_logger().info("✓ MotorHAT 初始化成功")
        except Exception as e:
            self.get_logger().error(f"✗ MotorHAT 初始化失敗: {e}")
            raise
        
        # 馬達參數
        self.rotation_speed = 120  # PWM 速度 (0-255)
        self.is_rotating = False
        
        # 訂閱相機節點的障礙物檢測訊息
        self.obstacle_sub = self.create_subscription(
            Bool,
            'obstacle_detected',  # 相機節點發布的話題
            self.obstacle_callback,
            10
        )
        
        # 訂閱距離訊息
        self.distance_sub = self.create_subscription(
            Float32,
            'obstacle_distance',  # 相機節點發布的距離
            self.distance_callback,
            10
        )
        
        # 發布馬達狀態
        self.motor_status_pub = self.create_publisher(Bool, 'motor_status', 10)
        
        # 定時發布狀態
        self.create_timer(1.0, self.publish_status)
        
        # 啟動時開始旋轉
        self.start_rotation()
        
        self.get_logger().info("馬達控制節點已啟動")
        self.get_logger().info("等待相機節點的障礙物檢測訊息...")
    
    def obstacle_callback(self, msg):
        """
        接收障礙物檢測訊息
        True = 有障礙物，停止馬達
        False = 無障礙物，開始旋轉
        """
        obstacle_detected = msg.data
        
        if obstacle_detected and self.is_rotating:
            # 有障礙物且正在旋轉 -> 停止
            self.stop_rotation()
            self.get_logger().info("⚠️  收到障礙物訊號，馬達停止")
            
        elif not obstacle_detected and not self.is_rotating:
            # 無障礙物且已停止 -> 開始旋轉
            self.start_rotation() 
            self.get_logger().info("✅ 障礙物消失，馬達開始旋轉")
    
    def distance_callback(self, msg):
        """
        接收距離訊息
        """
        distance = msg.data
        # 可以根據距離做更精細的控制
        if distance > 0:
            self.get_logger().info(f"距離障礙物: {distance:.2f} 公尺", throttle_duration_sec=2.0)
    
    def start_rotation(self):
        """開始兩個馬達同時順時針旋轉"""
        if not self.is_rotating:
            try:
                # 兩個馬達都設為相同方向（順時針）
                # 這裡假設 FORWARD 是順時針方向
                self.left_motor.setSpeed(self.rotation_speed)
                self.right_motor.setSpeed(self.rotation_speed)
                
                self.left_motor.run(Adafruit_MotorHAT.FORWARD)
                self.right_motor.run(Adafruit_MotorHAT.FORWARD)
                
                self.is_rotating = True
                self.get_logger().info(f"🔄 兩個馬達開始順時針旋轉 (速度: {self.rotation_speed})")
                
            except Exception as e:
                self.get_logger().error(f"啟動馬達失敗: {e}")
    
    def stop_rotation(self):
        """停止兩個馬達"""
        if self.is_rotating:
            try:
                self.left_motor.run(Adafruit_MotorHAT.RELEASE)
                self.right_motor.run(Adafruit_MotorHAT.RELEASE)
                
                self.is_rotating = False
                self.get_logger().info("⏹️ 兩個馬達已停止")
                
            except Exception as e:
                self.get_logger().error(f"停止馬達失敗: {e}")
    
    def publish_status(self):
        """發布馬達狀態"""
        status_msg = Bool()
        status_msg.data = self.is_rotating
        self.motor_status_pub.publish(status_msg)
    
    def destroy_node(self):
        """節點銷毀時安全停止馬達"""
        self.get_logger().info("正在安全停止馬達...")
        try:
            self.left_motor.run(Adafruit_MotorHAT.RELEASE)
            self.right_motor.run(Adafruit_MotorHAT.RELEASE)
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MotorNode()  # 使用上面第一個程式的內容
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()