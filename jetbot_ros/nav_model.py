import os
import sys
import math
import rclpy
import numpy as np
import PIL
import cv2
import threading
import time

from stereo_vision import StereoVision
from motors_nvidia import MotorController

from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray, String
from nav_msgs.msg import Odometry

from jetbot_ros.dnn.navigation_model import NavigationModel


class NavModelNode(Node):
    """
    Enhanced navigation model ROS node with obstacle avoidance and coordinate navigation
    """
    def __init__(self):
        super().__init__('nav_model', namespace='jetbot')
        
        self.setup_original_camera()
        self.setup_original_model()
        
        self.setup_dual_camera_mode()
        self.setup_obstacle_avoidance()
        self.setup_coordinate_navigation()
        
        self.setup_user_input()

    def setup_original_camera(self):
        """設定原有的單攝影機"""

        self.image_subscriber = self.create_subscription(
            Image, 'camera/image_raw', self.image_listener, 10)
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def setup_original_model(self):
        """設定原有的模型參數"""
        self.declare_parameter('model')
        self.declare_parameter('type', 'regression')
        self.declare_parameter('speed_gain', 0.15)
        self.declare_parameter('steering_gain', 0.4)
        self.declare_parameter('visualize', False)
        
        self.declare_parameter('dual_camera', False)
        self.declare_parameter('obstacle_avoidance', True)
        self.declare_parameter('coordinate_navigation', True)
        
        model_path = self.get_parameter('model').value
        model_type = self.get_parameter('type').value
        
        if model_path is not None:
            self.model = NavigationModel(model_path, type=model_type)
            self.get_logger().info(f"Loaded model: {model_path}")
        else:
            self.model = None
            self.get_logger().warning("No model specified, using obstacle avoidance only")

    def setup_dual_camera_mode(self):
        """設定雙攝影機模式"""
        self.dual_camera_enabled = self.get_parameter('dual_camera').value
        
        if self.dual_camera_enabled:
            # 雙攝影機訂閱
            self.left_subscriber = self.create_subscription(
                Image, 'camera_left/image_raw', self.left_camera_callback, 10)
            self.right_subscriber = self.create_subscription(
                Image, 'camera_right/image_raw', self.right_camera_callback, 10)
            
            # 攝影機影像儲存
            self.left_image = None
            self.right_image = None
            self.get_logger().info("Dual camera mode enabled")
        else:
            self.left_image = None
            self.right_image = None

    def setup_obstacle_avoidance(self):
        """設定避障功能"""
        self.obstacle_avoidance_enabled = self.get_parameter('obstacle_avoidance').value
        
        if self.obstacle_avoidance_enabled:
            # 避障相關參數
            self.regions = {
                'left': 1.0, 'fleft': 1.0, 'front': 1.0, 'fright': 1.0, 'right': 1.0
            }
            self.obstacle_threshold = 0.5
            self.is_avoiding = False
            
            # 避障狀態發布
            self.obstacle_detected_pub = self.create_publisher(Bool, 'obstacle_detected', 10)
            
            self.get_logger().info("Obstacle avoidance enabled")

    def setup_coordinate_navigation(self):
        """設定座標導航功能"""
        self.coord_nav_enabled = self.get_parameter('coordinate_navigation').value
        
        if self.coord_nav_enabled:
            # 導航相關狀態
            self.target_x = 0.0
            self.target_y = 0.0
            self.has_target = False
            self.navigation_active = False
            
            # 位置追蹤（簡化版，無里程計）
            self.current_x = 0.0
            self.current_y = 0.0
            self.current_direction = 0.0
            self.last_update_time = self.get_clock().now()
            
            # 導航參數
            self.base_speed = 0.2
            self.max_speed = 0.4
            self.arrival_threshold = 0.3
            
            # 狀態發布
            self.position_pub = self.create_publisher(Float32MultiArray, 'robot_position', 10)
            self.navigation_status_pub = self.create_publisher(String, 'navigation_status', 10)
            
            self.get_logger().info("Coordinate navigation enabled")

    def setup_user_input(self):
        """設定用戶輸入線程"""
        if self.coord_nav_enabled:
            self.should_stop = False
            self.input_thread = threading.Thread(target=self.user_input_thread, daemon=True)
            self.input_thread.start()

    def user_input_thread(self):
        """用戶輸入線程"""
        time.sleep(2.0)
        print("\n" + "="*50)
        print("座標導航模式已啟動")
        print("輸入目標座標 (格式: x y)，或 'exit' 結束")
        print("="*50)
        
        while not self.should_stop and rclpy.ok():
            try:
                user_input = input("目標座標: ").strip()
                
                if user_input.lower() in ['exit', 'quit']:
                    self.should_stop = True
                    break
                
                parts = user_input.split()
                if len(parts) == 2:
                    x, y = float(parts[0]), float(parts[1])
                    self.set_navigation_target(x, y)
                else:
                    print("格式錯誤！請輸入: x y")
                    
            except ValueError:
                print("請輸入有效數字！")
            except (EOFError, KeyboardInterrupt):
                self.should_stop = True
                break

    def set_navigation_target(self, x, y):
        """設置導航目標"""
        self.target_x = x
        self.target_y = y
        self.has_target = True
        self.navigation_active = True
        
        # 重置位置
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_direction = 0.0
        
        print(f"目標設定: ({x}, {y})")
        
        if hasattr(self, 'navigation_status_pub'):
            status_msg = String()
            status_msg.data = "NAVIGATING"
            self.navigation_status_pub.publish(status_msg)

    def left_camera_callback(self, msg):
        """左攝影機回調"""
        if self.dual_camera_enabled:
            try:
                self.left_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.update_obstacle_regions()
            except Exception as e:
                self.get_logger().error(f"Left camera error: {str(e)}")

    def right_camera_callback(self, msg):
        """右攝影機回調"""
        if self.dual_camera_enabled:
            try:
                self.right_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.update_obstacle_regions()
            except Exception as e:
                self.get_logger().error(f"Right camera error: {str(e)}")

    def update_obstacle_regions(self):
        """更新障礙物檢測區域"""
        if not self.obstacle_avoidance_enabled:
            return
            
        if self.dual_camera_enabled:
            if self.left_image is None or self.right_image is None:
                return
            # 使用雙攝影機進行檢測
            self.regions = self.model.detect_obstacles_dual_camera(
                self.left_image, self.right_image) if self.model else self.default_obstacle_detection()
        else:
            # 使用單攝影機檢測（在 image_listener 中處理）
            pass

    def default_obstacle_detection(self):
        """預設的障礙物檢測（當沒有模型時）"""
        # 簡化的障礙物檢測
        return {'left': 1.0, 'fleft': 1.0, 'front': 1.0, 'fright': 1.0, 'right': 1.0}

    def image_listener(self, msg):
        """原有的影像監聽器，增強功能"""
        self.get_logger().debug(f"received image: {msg.width}x{msg.height}, {msg.encoding}")

        if msg.encoding != 'rgb8' and msg.encoding != 'bgr8':
            raise ValueError(f"image encoding is '{msg.encoding}' (expected rgb8 or bgr8)")
            
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

        if msg.encoding == 'bgr8':
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # 計算所有導航向量
        dnn_twist = self.calculate_dnn_navigation(img)
        avoid_twist = self.calculate_avoidance_navigation(img)
        coord_twist = self.calculate_coordinate_navigation()
        
        # 融合向量
        final_twist = self.fuse_navigation_vectors(dnn_twist, avoid_twist, coord_twist)
        
        # 發布最終指令
        self.velocity_publisher.publish(final_twist)
        
        # 更新位置估算
        self.update_position_estimation(final_twist)
        
        # 發布狀態
        self.publish_status()

    def calculate_dnn_navigation(self, img):
        """計算 DNN 導航向量"""
        twist = Twist()
        
        if self.model is not None:
            # 原有的 DNN 推理邏輯
            xy = self.model.infer(PIL.Image.fromarray(img))
            
            x = xy[0]
            y = (0.5 - xy[1]) / 2.0
            
            steering_angle = np.arctan2(x, y)
            
            twist.linear.x = self.get_parameter('speed_gain').value
            twist.angular.z = -steering_angle * self.get_parameter('steering_gain').value
            
        return twist

    def calculate_avoidance_navigation(self, img):
        """計算避障導航向量"""
        twist = Twist()
        
        if self.obstacle_avoidance_enabled:
            if not self.dual_camera_enabled:
                # 單攝影機障礙物檢測
                if self.model:
                    self.regions = self.model.detect_obstacles_single_camera(img)
                else:
                    self.regions = self.default_obstacle_detection()
            
            # 計算避障向量
            if self.model:
                linear, angular = self.model.calculate_avoidance_vector(self.regions)
                twist.linear.x = linear
                twist.angular.z = angular
            
        return twist

    def calculate_coordinate_navigation(self):
        """計算座標導航向量"""
        twist = Twist()
        
        if self.coord_nav_enabled and self.has_target and self.navigation_active:
            dx = self.target_x - self.current_x
            dy = self.target_y - self.current_y
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance < self.arrival_threshold:
                self.navigation_active = False
                self.has_target = False
                print(f"已到達目標！距離: {distance:.3f}m")
                return twist
            
            # 計算導航指令
            target_angle = math.degrees(math.atan2(dy, dx))
            angle_error = self.normalize_angle(target_angle - self.current_direction)
            
            twist.linear.x = min(self.base_speed, self.max_speed)
            twist.angular.z = math.radians(angle_error) * 0.5
            
        return twist

    def fuse_navigation_vectors(self, dnn_twist, avoid_twist, coord_twist):
        """融合多個導航向量"""
        final_twist = Twist()
        
        # 動態權重計算
        dnn_weight = 0.4 if self.model else 0.0
        avoid_weight = 0.4 if self.obstacle_avoidance_enabled and self.is_avoiding else 0.0
        coord_weight = 0.6 if self.coord_nav_enabled and self.has_target else 0.0
        
        # 如果沒有座標導航，提高 DNN 權重
        if not self.has_target:
            dnn_weight = 0.8 if self.model else 0.0
            avoid_weight = 0.2
            coord_weight = 0.0
        
        # 標準化權重
        total_weight = dnn_weight + avoid_weight + coord_weight
        if total_weight > 0:
            dnn_weight /= total_weight
            avoid_weight /= total_weight
            coord_weight /= total_weight
        else:
            # 預設使用 DNN
            dnn_weight = 1.0
        
        # 融合
        final_twist.linear.x = (dnn_twist.linear.x * dnn_weight +
                               avoid_twist.linear.x * avoid_weight +
                               coord_twist.linear.x * coord_weight)
        
        final_twist.angular.z = (dnn_twist.angular.z * dnn_weight +
                                avoid_twist.angular.z * avoid_weight +
                                coord_twist.angular.z * coord_weight)
        
        # 安全限制
        final_twist.linear.x = max(-0.3, min(0.4, final_twist.linear.x))
        final_twist.angular.z = max(-0.6, min(0.6, final_twist.angular.z))
        
        return final_twist

    def update_position_estimation(self, twist):
        """更新位置估算"""
        if not self.coord_nav_enabled:
            return
            
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = current_time
        
        if dt > 0.5:  # 防止異常
            return
        
        # 更新方向和位置
        self.current_direction += math.degrees(twist.angular.z * dt)
        self.current_direction = self.normalize_angle(self.current_direction)
        
        direction_rad = math.radians(self.current_direction)
        self.current_x += twist.linear.x * math.cos(direction_rad) * dt
        self.current_y += twist.linear.x * math.sin(direction_rad) * dt

    def normalize_angle(self, angle):
        """角度標準化"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def publish_status(self):
        """發布狀態信息"""
        # 發布障礙物檢測狀態
        if self.obstacle_avoidance_enabled and hasattr(self, 'obstacle_detected_pub'):
            obstacle_msg = Bool()
            obstacle_msg.data = self.is_avoiding
            self.obstacle_detected_pub.publish(obstacle_msg)
        
        # 發布位置信息
        if self.coord_nav_enabled and hasattr(self, 'position_pub'):
            position_msg = Float32MultiArray()
            position_msg.data = [self.current_x, self.current_y, self.current_direction]
            self.position_pub.publish(position_msg)

    def destroy_node(self):
        """節點銷毀時的清理工作"""
        if hasattr(self, 'should_stop'):
            self.should_stop = True
            
        # 停止機器人
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        
        self.get_logger().info('shutting down, stopping robot...')
        self.velocity_publisher.publish(twist)
        
        super().destroy_node()
        

def main(args=None):
    rclpy.init(args=args)
    node = NavModelNode()
    node.get_logger().info("Enhanced navigation node started...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()