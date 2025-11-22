#!/usr/bin/env python3
"""
雙相機物體檢測節點 - Stereo Vision Node
功能：
1. 直接讀取雙 CSI 相機影像
2. 檢測車子前方是否有物體
3. 計算距離障礙物的距離
4. 發送訊息給馬達節點
"""
import rclpy
import cv2
import numpy as np
import threading
import time
import yaml
import os
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory


class CameraNode(Node):
    """
    雙相機物體檢測節點 - 直接讀取物理相機
    """
    
    def __init__(self):
        super().__init__('camera_detection_node')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 影像儲存
        self.left_image = None
        self.right_image = None
        self.image_lock = threading.Lock()
        
        self.camera_left = None
        self.camera_right = None
        self.capture_thread = None
        self.running = False
        
        # 參數宣告
        self.declare_parameter('obstacle_threshold', 2000)
        self.declare_parameter('detection_area_ratio', 0.6)
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('baseline_mm', 60)  
        self.declare_parameter('focal_length', 400)  
        
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.detection_area_ratio = self.get_parameter('detection_area_ratio').value
        self.debug_mode = self.get_parameter('debug_mode').value
        self.baseline_mm = self.get_parameter('baseline_mm').value
        self.focal_length = self.get_parameter('focal_length').value
        
        # 發布器
        self.obstacle_pub = self.create_publisher(Bool, 'obstacle_detected', 10)
        self.distance_pub = self.create_publisher(Float32, 'obstacle_distance', 10)

        self.left_image_pub = self.create_publisher(Image, '/camera_left/image_raw', 10)
        self.right_image_pub = self.create_publisher(Image, '/camera_right/image_raw', 10)

        # Camera Info 發布器
        self.left_info_pub = self.create_publisher(CameraInfo, '/camera_left/camera_info', 10)
        self.right_info_pub = self.create_publisher(CameraInfo, '/camera_right/camera_info', 10)

        # 載入校準參數
        self.left_camera_info = CameraInfo()
        self.right_camera_info = CameraInfo()
        self.load_camera_calibration()

        # 定時處理 (10Hz)
        self.create_timer(0.1, self.process_images)

        # 立體視覺匹配器
        self.stereo_matcher = cv2.StereoBM_create(numDisparities=64, blockSize=15)

        # 初始化相機
        self.init_physical_cameras()
        
    
    def load_camera_calibration(self):
        """從 YAML 檔案載入相機校準參數"""
        try:
            pkg_share = get_package_share_directory('jetbot_ros')
            left_yaml = os.path.join(pkg_share, 'config', 'left.yaml')
            right_yaml = os.path.join(pkg_share, 'config', 'right.yaml')

            # 載入左相機校準
            if os.path.exists(left_yaml):
                with open(left_yaml, 'r') as f:
                    left_calib = yaml.safe_load(f)
                self.left_camera_info = self.yaml_to_camera_info(left_calib, 'camera_left')
                self.get_logger().info(f"載入左相機校準: {left_yaml}")
            else:
                self.get_logger().warn(f"左相機校準檔案不存在: {left_yaml}，使用預設參數")
                self.left_camera_info = self.get_default_camera_info('camera_left')

            # 載入右相機校準
            if os.path.exists(right_yaml):
                with open(right_yaml, 'r') as f:
                    right_calib = yaml.safe_load(f)
                self.right_camera_info = self.yaml_to_camera_info(right_calib, 'camera_right')
                self.get_logger().info(f"載入右相機校準: {right_yaml}")
            else:
                self.get_logger().warn(f"右相機校準檔案不存在: {right_yaml}，使用預設參數")
                self.right_camera_info = self.get_default_camera_info('camera_right')

        except Exception as e:
            self.get_logger().error(f"載入校準參數失敗: {e}")
            self.left_camera_info = self.get_default_camera_info('camera_left')
            self.right_camera_info = self.get_default_camera_info('camera_right')

    def yaml_to_camera_info(self, calib_data, frame_id):
        """將 YAML 校準資料轉換為 CameraInfo 消息"""
        info = CameraInfo()
        info.header.frame_id = frame_id
        info.width = calib_data.get('image_width', 640)
        info.height = calib_data.get('image_height', 480)
        info.distortion_model = calib_data.get('distortion_model', 'plumb_bob')

        # Camera matrix
        if 'camera_matrix' in calib_data:
            cm = calib_data['camera_matrix']
            info.k = cm.get('data', [0.0]*9)

        # Distortion coefficients
        if 'distortion_coefficients' in calib_data:
            dc = calib_data['distortion_coefficients']
            info.d = dc.get('data', [0.0]*5)

        # Rectification matrix
        if 'rectification_matrix' in calib_data:
            rm = calib_data['rectification_matrix']
            info.r = rm.get('data', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])

        # Projection matrix
        if 'projection_matrix' in calib_data:
            pm = calib_data['projection_matrix']
            info.p = pm.get('data', [0.0]*12)

        return info

    def get_default_camera_info(self, frame_id):
        """取得預設相機參數（未校準）"""
        info = CameraInfo()
        info.header.frame_id = frame_id
        info.width = 640
        info.height = 480
        info.distortion_model = 'plumb_bob'

        # 預設內參矩陣（估計值）
        fx = 500.0  # 焦距
        fy = 500.0
        cx = 320.0  # 主點
        cy = 240.0

        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        return info

    def init_physical_cameras(self):
        """直接初始化物理 CSI 相機"""
        try:
            self.get_logger().info("正在初始化雙 CSI 相機...")

            # 左相機 (CSI-1) - 交換後
            gst_left = (
                "nvarguscamerasrc sensor_id=1 ! "
                "video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! "
                "nvvidconv ! video/x-raw, format=BGRx ! "
                "videoconvert ! video/x-raw, format=BGR ! "
                "appsink drop=1 max-buffers=2"
            )
            self.camera_left = cv2.VideoCapture(gst_left, cv2.CAP_GSTREAMER)

            # 右相機 (CSI-0) - 交換後
            gst_right = (
                "nvarguscamerasrc sensor_id=0 ! "
                "video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! "
                "nvvidconv ! video/x-raw, format=BGRx ! "
                "videoconvert ! video/x-raw, format=BGR ! "
                "appsink drop=1 max-buffers=2"
            )
            self.camera_right = cv2.VideoCapture(gst_right, cv2.CAP_GSTREAMER)

            if not self.camera_left.isOpened():
                raise RuntimeError("無法開啟左相機 (CSI-1)")
            if not self.camera_right.isOpened():
                raise RuntimeError("無法開啟右相機 (CSI-0)")
                
            self.get_logger().info("✓ 雙相機初始化成功")
            
            # 啟動擷取執行緒
            self.running = True
            self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
            self.capture_thread.start()
            
        except Exception as e:
            self.get_logger().error(f"相機初始化失敗: {e}")
            raise
    
    
    def capture_loop(self):
        """持續擷取影像的執行緒"""
        while self.running:
            ret_left, frame_left = self.camera_left.read()
            ret_right, frame_right = self.camera_right.read()
            
            if ret_left and ret_right:
                with self.image_lock:
                    self.left_image = frame_left.copy()
                    self.right_image = frame_right.copy()
            
            time.sleep(0.01)  # 避免過度佔用 CPU
    
    
    def process_images(self):
        """處理影像並進行障礙物檢測"""
        with self.image_lock:
            if self.left_image is None or self.right_image is None:
                return
            
            left_img = self.left_image.copy()
            right_img = self.right_image.copy()
        
        # 發布原始影像
        try:
            left_msg = self.bridge.cv2_to_imgmsg(left_img, encoding='bgr8')
            right_msg = self.bridge.cv2_to_imgmsg(right_img, encoding='bgr8')
            
            left_msg.header.stamp = self.get_clock().now().to_msg()
            right_msg.header.stamp = self.get_clock().now().to_msg()
            
            left_msg.header.frame_id = 'camera_left'
            right_msg.header.frame_id = 'camera_right'
            
            self.left_image_pub.publish(left_msg)
            self.right_image_pub.publish(right_msg)

            # 發布 Camera Info
            self.left_camera_info.header.stamp = left_msg.header.stamp
            self.right_camera_info.header.stamp = right_msg.header.stamp
            self.left_info_pub.publish(self.left_camera_info)
            self.right_info_pub.publish(self.right_camera_info)
        except Exception as e:
            self.get_logger().error(f"影像發布失敗: {e}")
            return
        
        # 障礙物檢測
        obstacle_detected = self.detect_obstacle_simple(left_img)
        
        # 發布檢測結果
        obs_msg = Bool()
        obs_msg.data = obstacle_detected
        self.obstacle_pub.publish(obs_msg)
        
        if self.debug_mode:
            self.get_logger().info(f"障礙物檢測: {'有' if obstacle_detected else '無'}")
    
    
    def detect_obstacle_simple(self, image):
        """簡單的障礙物檢測 - 使用邊緣檢測"""
        # 轉灰階
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 定義檢測區域 (中央下方區域)
        h, w = gray.shape
        roi_top = int(h * (1 - self.detection_area_ratio))
        roi = gray[roi_top:, :]
        
        # 邊緣檢測
        edges = cv2.Canny(roi, 50, 150)
        
        # 計算邊緣像素數
        edge_count = np.count_nonzero(edges)
        
        # 判斷是否有障礙物
        return edge_count > self.obstacle_threshold
    
    
    def compute_stereo_disparity(self, left_img, right_img):
        """計算立體視差圖"""
        # 轉灰階
        gray_left = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
        
        # 計算視差
        disparity = self.stereo_matcher.compute(gray_left, gray_right)
        
        return disparity
    
    
    def estimate_distance(self, disparity):
        """從視差估算距離 (單位: 公分)"""
        # 距離公式: distance = (baseline * focal_length) / disparity
        # baseline: 兩相機間距 (mm)
        # focal_length: 焦距 (pixels)
        
        # 取中央區域的平均視差
        h, w = disparity.shape
        center_region = disparity[h//3:2*h//3, w//3:2*w//3]
        
        # 過濾無效視差
        valid_disparity = center_region[center_region > 0]
        
        if len(valid_disparity) == 0:
            return float('inf')
        
        avg_disparity = np.median(valid_disparity)
        
        # 計算距離 (轉換為公分)
        distance_mm = (self.baseline_mm * self.focal_length) / (avg_disparity / 16.0)
        distance_cm = distance_mm / 10.0
        
        return distance_cm
    
    
    def destroy_node(self):
        """清理資源"""
        self.running = False
        
        if self.capture_thread:
            self.capture_thread.join(timeout=1.0)
        
        if self.camera_left:
            self.camera_left.release()
        if self.camera_right:
            self.camera_right.release()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = CameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()