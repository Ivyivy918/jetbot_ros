#!/usr/bin/env python3
"""
雙相機物體檢測節點
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
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge

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
        
        self.obstacle_pub = self.create_publisher(Bool, 'obstacle_detected', 10)
        self.distance_pub = self.create_publisher(Float32, 'obstacle_distance', 10)
        
        self.left_image_pub = self.create_publisher(Image, '/camera_left/image_raw', 10)
        self.right_image_pub = self.create_publisher(Image, '/camera_right/image_raw', 10)
        
        self.create_timer(0.1, self.process_images)  # 10Hz
        
        self.stereo_matcher = cv2.StereoBM_create(numDisparities=64, blockSize=15)
        
        self.init_physical_cameras()
        
    
    def init_physical_cameras(self):
        """直接初始化物理 CSI 相機"""
        try:
            self.get_logger().info("正在初始化雙 CSI 相機...")
            
            # 左相機 (CSI-0)
            gst_left = (
                "nvarguscamerasrc sensor_id=0 ! "
                "video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! "
                "nvvidconv ! video/x-raw, format=BGRx ! "
                "videoconvert ! video/x-raw, format=BGR ! "
                "appsink drop=1 max-buffers=2"
            )
            self.camera_left = cv2.VideoCapture(gst_left, cv2.CAP_GSTREAMER)
            
            # 右相機 (CSI-1)
            gst_right = (
                "nvarguscamerasrc sensor_id=1 ! "
                "video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! "
                "nvvidconv ! video/x-raw, format=BGRx ! "
                "videoconvert ! video/x-raw, format=BGR ! "
                "appsink drop=1 max-buffers=2"
            )
            self.camera_right = cv2.VideoCapture(gst_right, cv2.CAP_GSTREAMER)
            
            # 檢查相機是否成功開啟
            if self.camera_left.isOpened() and self.camera_right.isOpened():
                self.get_logger().info("✓ 雙 CSI 相機初始化成功")
                self.start_capture_thread()
            elif self.camera_left.isOpened():
                self.get_logger().warning("⚠ 只有左相機可用，將使用單相機模式")
                self.start_capture_thread()
            else:
                self.get_logger().error("✗ 無法初始化任何相機")
                
        except Exception as e:
            self.get_logger().error(f"相機初始化錯誤: {e}")
    
    def start_capture_thread(self):
        """啟動相機擷取線程"""
        self.running = True
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()
        self.get_logger().info("相機擷取線程已啟動")
    
    def capture_loop(self):
        """相機擷取迴圈"""
        while self.running:
            try:
                frames_captured = False
                
                # 讀取左相機
                if self.camera_left and self.camera_left.isOpened():
                    ret_left, frame_left = self.camera_left.read()
                    if ret_left:
                        with self.image_lock:
                            self.left_image = frame_left
                            frames_captured = True
                        
                        # 發布左相機影像 (可選)
                        if hasattr(self, 'left_image_pub'):
                            try:
                                img_msg = self.bridge.cv2_to_imgmsg(frame_left, 'bgr8')
                                self.left_image_pub.publish(img_msg)
                            except:
                                pass
                
                # 讀取右相機  
                if self.camera_right and self.camera_right.isOpened():
                    ret_right, frame_right = self.camera_right.read()
                    if ret_right:
                        with self.image_lock:
                            self.right_image = frame_right
                            frames_captured = True
                        
                        # 發布右相機影像 (可選)
                        if hasattr(self, 'right_image_pub'):
                            try:
                                img_msg = self.bridge.cv2_to_imgmsg(frame_right, 'bgr8')
                                self.right_image_pub.publish(img_msg)
                            except:
                                pass
                
                if not frames_captured:
                    self.get_logger().warning("無法讀取相機影像")
                    
            except Exception as e:
                self.get_logger().error(f"相機擷取錯誤: {e}")
                break
            
            time.sleep(0.033)  # ~30 FPS
    
    def process_images(self):
        """處理雙相機影像並發送檢測結果給馬達節點"""
        with self.image_lock:
            if self.left_image is None:
                # 沒有影像時發布 "無障礙物" 狀態，讓馬達繼續轉動
                self.publish_detection_results(False, -1.0)
                return
            
            left_img = self.left_image.copy()
            right_img = self.right_image.copy() if self.right_image is not None else None
        
        try:
            # 計算距離 (使用改進的方法)
            distance_cm = self.calculate_distance_precise(left_img, right_img)
            
            # **停止閾值設為15公分**
            obstacle_detected = bool(distance_cm > 0 and distance_cm <= 15.0)
            
            # **簡潔的狀態顯示**
            if distance_cm > 0:
                # 只有距離小於50公分時才顯示距離
                if distance_cm < 50.0:
                    if obstacle_detected:
                        print(f"障礙物距離: {distance_cm:.1f} cm - 馬達停止")
                    else:
                        print(f"障礙物距離: {distance_cm:.1f} cm - 馬達運行")
                else:
                    # 距離>=50公分時不顯示距離，只顯示狀態
                    print("馬達運行")
            else:
                print("馬達運行")
                obstacle_detected = False
            
            # **發布檢測結果給馬達節點**
            self.publish_detection_results(obstacle_detected, float(distance_cm / 100.0) if distance_cm > 0 else -1.0)
            
            # 除錯模式：顯示影像
            if self.debug_mode:
                self.show_debug_images(left_img, right_img, obstacle_detected, distance_cm / 100.0 if distance_cm > 0 else -1.0)
                
        except Exception as e:
            print("馬達運行")
            # 發生錯誤時也要發布狀態，避免馬達卡住
            self.publish_detection_results(False, -1.0)
    
    def detect_front_obstacle(self, left_image, right_image):
        """
        檢測車子前方是否有物體
        使用左右相機的邊緣檢測結果（如果有雙相機）或單相機檢測
        """
        try:
            obstacle_left = self.detect_obstacle_single_camera(left_image)
            
            if right_image is not None:
                # 雙相機模式：兩個相機都檢測到物體才判定為有障礙物
                obstacle_right = self.detect_obstacle_single_camera(right_image)
                has_obstacle = obstacle_left and obstacle_right
            else:
                # 單相機模式：只使用左相機
                has_obstacle = obstacle_left
            
            return has_obstacle
            
        except Exception as e:
            self.get_logger().error(f"前方物體檢測錯誤: {e}")
            return False
    
    def detect_obstacle_single_camera(self, image):
        """單一相機的物體檢測"""
        try:
            height, width = image.shape[:2]
            
            # 定義前方檢測區域
            roi_y_start = int(height * self.detection_area_ratio)
            roi_x_start = int(width * 0.25)  # 前方中央區域
            roi_x_end = int(width * 0.75)
            
            # 擷取感興趣區域
            roi = image[roi_y_start:height, roi_x_start:roi_x_end]
            
            # 邊緣檢測
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blurred, 50, 150)
            
            # 計算邊緣像素數量
            edge_count = cv2.countNonZero(edges)
            
            return edge_count > self.obstacle_threshold
            
        except Exception as e:
            self.get_logger().error(f"單相機檢測錯誤: {e}")
            return False
    
    def calculate_distance_precise(self, left_image, right_image):
        """
        更精確的距離計算，返回公分為單位
        修正計算邏輯，避免異常大的距離值
        """
        distance_cm = -1.0
        
        # 方法1：基於輪廓面積的距離估算 (更穩定)
        try:
            height, width = left_image.shape[:2]
            
            # 檢測區域 (前方中央)
            roi_y_start = int(height * 0.5)  # 從中間往下檢測
            roi_x_start = int(width * 0.3)
            roi_x_end = int(width * 0.7)
            roi = left_image[roi_y_start:height, roi_x_start:roi_x_end]
            
            # 邊緣檢測
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blurred, 30, 100)
            
            # 找輪廓
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # 找最大輪廓
                max_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(max_contour)
                
                # **修正：改進距離計算公式，避免異常大的值**
                if area > 50:  # 最小面積閾值
                    # 使用更合理的距離計算公式
                    if area > 5000:  # 非常大的輪廓 = 很近
                        distance_cm = 5.0
                    elif area > 2000:  # 大輪廓 = 近
                        distance_cm = 10.0
                    elif area > 1000:  # 中等輪廓 = 中距離
                        distance_cm = 20.0
                    elif area > 500:   # 小輪廓 = 遠距離
                        distance_cm = 50.0
                    else:              # 很小輪廓 = 很遠
                        distance_cm = 100.0
            
        except Exception as e:
            self.get_logger().debug(f"輪廓檢測處理中: {e}")
        
        # 方法2：如果有雙相機，嘗試立體視覺 (備用)
        if distance_cm <= 0 and right_image is not None:
            try:
                stereo_distance = self.calculate_distance_stereo(left_image, right_image)
                if stereo_distance > 0:
                    distance_cm = stereo_distance * 100  # 轉為公分
                    # **限制距離範圍，避免異常值**
                    distance_cm = max(5.0, min(200.0, distance_cm))
            except Exception as e:
                self.get_logger().debug(f"立體視覺計算中: {e}")
        
        # 方法3：基於邊緣密度的粗略估算 (最後備用)
        if distance_cm <= 0:
            try:
                height, width = left_image.shape[:2]
                roi = left_image[int(height * 0.6):height, int(width * 0.3):int(width * 0.7)]
                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                edges = cv2.Canny(gray, 50, 150)
                edge_count = cv2.countNonZero(edges)
                
                # **修正：使用更合理的邊緣密度計算**
                if edge_count > 2000:      # 很多邊緣 = 很近
                    distance_cm = 8.0
                elif edge_count > 1000:    # 多邊緣 = 近
                    distance_cm = 15.0
                elif edge_count > 500:     # 中等邊緣 = 中距離
                    distance_cm = 30.0
                elif edge_count > 200:     # 少邊緣 = 遠
                    distance_cm = 60.0
                elif edge_count > 50:      # 很少邊緣 = 很遠
                    distance_cm = 120.0
                    
            except Exception as e:
                self.get_logger().debug(f"邊緣密度分析中: {e}")
        
        # **最終檢查：確保距離在合理範圍內**
        if distance_cm > 0:
            distance_cm = max(3.0, min(300.0, distance_cm))  # 限制在 3-300 公分之間
        
        return distance_cm if distance_cm > 0 else -1.0
    
    def calculate_distance_stereo(self, left_image, right_image):
        """
        立體視覺距離計算 (改進版)
        """
        try:
            # 轉換為灰階
            gray_left = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
            gray_right = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)
            
            # 改進立體匹配參數
            stereo = cv2.StereoBM_create(numDisparities=48, blockSize=11)
            stereo.setMinDisparity(4)
            stereo.setUniquenessRatio(5)
            stereo.setSpeckleWindowSize(50)
            stereo.setSpeckleRange(16)
            
            # 計算視差
            disparity = stereo.compute(gray_left, gray_right)
            
            # 取中央區域
            height, width = disparity.shape
            center_y = height // 2
            center_x = width // 2
            roi_size = 30
            
            center_disparity = disparity[
                center_y-roi_size:center_y+roi_size,
                center_x-roi_size:center_x+roi_size
            ]
            
            # 過濾有效視差
            valid_disp = center_disparity[(center_disparity > 0) & (center_disparity < 400)]
            
            if len(valid_disp) < 10:
                return -1.0
            
            # 使用中位數而不是平均值，更穩定
            avg_disparity = np.median(valid_disp) / 16.0
            
            if avg_disparity <= 0.1:
                return -1.0
            
            # 使用調整後的參數
            focal_length_adjusted = 150  # 經過實測調整的焦距
            baseline_mm = 60  # 6公分基線
            
            distance_mm = (baseline_mm * focal_length_adjusted) / avg_disparity
            distance_m = distance_mm / 1000.0
            
            # 合理範圍檢查
            if 0.05 <= distance_m <= 3.0:  # 5公分到3公尺
                return distance_m
            else:
                return -1.0
                
        except Exception as e:
            self.get_logger().debug(f"立體視覺處理中: {e}")
            return -1.0
    
    def publish_detection_results(self, obstacle_detected, distance):
        """發布檢測結果給馬達節點 - 修正數據類型問題"""
        try:
            # **修正：確保傳入的 obstacle_detected 是 bool 類型**
            obstacle_msg = Bool()
            obstacle_msg.data = bool(obstacle_detected)  # 明確轉換為 bool
            self.obstacle_pub.publish(obstacle_msg)
            
            # **修正：確保傳入的 distance 是 float 類型**
            distance_msg = Float32()
            distance_msg.data = float(distance) if distance > 0 else -1.0  # 明確轉換為 float
            self.distance_pub.publish(distance_msg)
            
        except Exception as e:
            # 移除錯誤訊息輸出，保持簡潔
            pass
    
    def show_debug_images(self, left_image, right_image, obstacle_detected, distance):
        """顯示除錯影像"""
        try:
            # 在左影像上繪製檢測區域和結果
            debug_left = left_image.copy()
            height, width = debug_left.shape[:2]
            
            # 繪製檢測區域
            roi_y_start = int(height * self.detection_area_ratio)
            roi_x_start = int(width * 0.25)
            roi_x_end = int(width * 0.75)
            
            color = (0, 0, 255) if obstacle_detected else (0, 255, 0)
            cv2.rectangle(debug_left, (roi_x_start, roi_y_start), 
                         (roi_x_end, height), color, 2)
            
            # 顯示檢測結果
            status_text = "OBSTACLE DETECTED" if obstacle_detected else "CLEAR"
            cv2.putText(debug_left, status_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            
            # 顯示距離
            if distance > 0:
                distance_text = f"Distance: {distance:.2f}m"
                cv2.putText(debug_left, distance_text, (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # 組合左右影像顯示
            if right_image is not None:
                combined = cv2.hconcat([debug_left, right_image])
                cv2.imshow('Dual Camera Detection', combined)
            else:
                cv2.imshow('Single Camera Detection', debug_left)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"除錯影像顯示錯誤: {e}")
    
    def cleanup(self):
        self.running = False
        
        if self.capture_thread:
            self.capture_thread.join(timeout=1)
        
        if self.camera_left:
            self.camera_left.release()
        if self.camera_right:
            self.camera_right.release()
        
        cv2.destroyAllWindows()
        
    def destroy_node(self):
        self.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CameraNode()
        print("=== 雙相機檢測節點 ===")
        print("-" * 40)
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n收到停止信號")
    except Exception as e:
        print(f"執行錯誤: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()