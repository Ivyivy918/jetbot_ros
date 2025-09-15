#!/usr/bin/env python3
"""
簡化版障礙物檢測程式
功能：
1. 車子啟動後持續轉動
2. 雙攝影機檢測障礙物
3. 偵測到障礙物立即停止馬達
"""

import cv2
import numpy as np
import time
import threading
import signal
import sys
import atexit
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

class SimpleObstacleDetection:
    def __init__(self, motor_i2c_addr=0x60):
        """
        初始化簡化的障礙物檢測系統
        """
        print("=== 簡化版障礙物檢測系統 ===")
        
        # 初始化馬達控制
        try:
            self.mh = Adafruit_MotorHAT(addr=motor_i2c_addr)
            self.left_motor = self.mh.getMotor(1)   # M1
            self.right_motor = self.mh.getMotor(2)  # M2
            print("✓ 馬達控制器初始化成功")
        except Exception as e:
            print(f"✗ 馬達控制器初始化失敗: {e}")
            sys.exit(1)
        
        # 初始化雙攝影機
        self.camera_left = None
        self.camera_right = None
        self.frame_left = None
        self.frame_right = None
        self.frame_lock = threading.Lock()
        
        # 系統狀態
        self.running = False
        self.obstacle_detected = False
        self.continuous_rotation = True  # 持續轉動模式
        
        # 運動參數
        self.rotation_speed = 100  # 轉動速度 (0-255)
        self.check_interval = 0.1  # 檢測間隔（秒）
        
        # 障礙物檢測參數
        self.obstacle_threshold = 2000  # 邊緣檢測閾值
        self.detection_area_ratio = 0.6  # 檢測區域比例
        
        # 註冊清理函數
        atexit.register(self.cleanup)
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # 初始化攝影機
        if not self.init_cameras():
            print("攝影機初始化失敗")
            sys.exit(1)
    
    def signal_handler(self, signum, frame):
        """處理中斷信號"""
        print("\n收到中斷信號，正在安全停止...")
        self.stop_system()
        sys.exit(0)
    
    def init_cameras(self):
        """初始化雙攝影機"""
        print("初始化雙攝影機...")
        
        try:
            # 左攝影機 (CSI-0)
            gst_left = (
                "nvarguscamerasrc sensor_id=0 ! "
                "video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! "
                "nvvidconv ! video/x-raw, format=BGRx ! "
                "videoconvert ! video/x-raw, format=BGR ! "
                "appsink drop=1 max-buffers=2"
            )
            
            self.camera_left = cv2.VideoCapture(gst_left, cv2.CAP_GSTREAMER)
            
            if self.camera_left.isOpened():
                print("✓ 左攝影機初始化成功")
            else:
                print("✗ 左攝影機初始化失敗")
                return False
            
            # 右攝影機 (CSI-1)
            gst_right = (
                "nvarguscamerasrc sensor_id=1 ! "
                "video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! "
                "nvvidconv ! video/x-raw, format=BGRx ! "
                "videoconvert ! video/x-raw, format=BGR ! "
                "appsink drop=1 max-buffers=2"
            )
            
            self.camera_right = cv2.VideoCapture(gst_right, cv2.CAP_GSTREAMER)
            
            if self.camera_right.isOpened():
                print("✓ 右攝影機初始化成功")
            else:
                print("✗ 右攝影機初始化失敗")
                return False
            
            return True
            
        except Exception as e:
            print(f"攝影機初始化錯誤: {e}")
            return False
    
    def start_rotation(self):
        """開始持續轉動"""
        if self.continuous_rotation:
            self.left_motor.setSpeed(self.rotation_speed)
            self.right_motor.setSpeed(self.rotation_speed)
            
            # 設定為原地右轉 (左輪前進，右輪後退)
            self.left_motor.run(Adafruit_MotorHAT.FORWARD)
            self.right_motor.run(Adafruit_MotorHAT.BACKWARD)
            
            print(f"開始原地轉動 (速度: {self.rotation_speed})")
    
    def stop_motors(self):
        """停止所有馬達"""
        self.left_motor.run(Adafruit_MotorHAT.RELEASE)
        self.right_motor.run(Adafruit_MotorHAT.RELEASE)
        print("馬達已停止")
    
    def capture_frames(self):
        """攝影機擷取線程"""
        while self.running:
            try:
                with self.frame_lock:
                    # 同時擷取兩個攝影機
                    if self.camera_left and self.camera_left.isOpened():
                        ret_left, frame_left = self.camera_left.read()
                        if ret_left:
                            self.frame_left = frame_left
                    
                    if self.camera_right and self.camera_right.isOpened():
                        ret_right, frame_right = self.camera_right.read()
                        if ret_right:
                            self.frame_right = frame_right
                
                time.sleep(0.033)  # ~30 FPS
                
            except Exception as e:
                print(f"攝影機擷取錯誤: {e}")
                break
    
    def detect_obstacle_simple(self):
        """
        簡化的障礙物檢測
        使用邊緣檢測來判斷是否有障礙物
        """
        with self.frame_lock:
            if self.frame_left is None and self.frame_right is None:
                return False
        
        # 使用左攝影機進行主要檢測
        frame = self.frame_left if self.frame_left is not None else self.frame_right
        
        if frame is None:
            return False
        
        try:
            height, width = frame.shape[:2]
            
            # 只檢測前方區域 (下半部分中央區域)
            roi_y_start = int(height * self.detection_area_ratio)
            roi_x_start = int(width * 0.3)
            roi_x_end = int(width * 0.7)
            
            roi = frame[roi_y_start:height, roi_x_start:roi_x_end]
            
            # 轉換為灰階
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            
            # 高斯模糊
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # Canny邊緣檢測
            edges = cv2.Canny(blurred, 50, 150)
            
            # 計算邊緣像素數量
            edge_count = cv2.countNonZero(edges)
            
            # 判斷是否有障礙物
            has_obstacle = edge_count > self.obstacle_threshold
            
            # 如果啟用調試模式，顯示檢測區域
            if hasattr(self, 'debug_mode') and self.debug_mode:
                debug_frame = frame.copy()
                cv2.rectangle(debug_frame, (roi_x_start, roi_y_start), 
                            (roi_x_end, height), (0, 255, 0) if not has_obstacle else (0, 0, 255), 2)
                cv2.putText(debug_frame, f"Edges: {edge_count}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.putText(debug_frame, f"Obstacle: {has_obstacle}", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255) if has_obstacle else (0, 255, 0), 2)
                
                cv2.imshow('Obstacle Detection', debug_frame)
                cv2.waitKey(1)
            
            return has_obstacle
            
        except Exception as e:
            print(f"障礙物檢測錯誤: {e}")
            return False
    
    def control_loop(self):
        """主控制循環"""
        print("開始主控制循環...")
        
        # 開始轉動
        self.start_rotation()
        
        while self.running:
            try:
                # 檢測障礙物
                obstacle_detected = self.detect_obstacle_simple()
                
                if obstacle_detected != self.obstacle_detected:
                    self.obstacle_detected = obstacle_detected
                    
                    if obstacle_detected:
                        print("⚠️  偵測到障礙物！停止馬達")
                        self.stop_motors()
                        self.continuous_rotation = False
                    else:
                        print("✓ 障礙物消失，恢復轉動")
                        self.continuous_rotation = True
                        self.start_rotation()
                
                time.sleep(self.check_interval)
                
            except Exception as e:
                print(f"控制循環錯誤: {e}")
                break
    
    def run(self, debug_mode=False):
        """
        啟動系統
        
        Args:
            debug_mode: 是否啟用調試模式（顯示檢測視窗）
        """
        print("啟動簡化版障礙物檢測系統...")
        print("功能：持續轉動 → 偵測障礙物 → 自動停止")
        print("按 Ctrl+C 退出")
        print("-" * 40)
        
        self.debug_mode = debug_mode
        self.running = True
        
        try:
            # 啟動攝影機擷取線程
            capture_thread = threading.Thread(target=self.capture_frames, daemon=True)
            capture_thread.start()
            
            # 等待攝影機穩定
            time.sleep(2)
            
            # 執行主控制循環
            self.control_loop()
            
        except KeyboardInterrupt:
            print("\n收到鍵盤中斷")
        except Exception as e:
            print(f"系統錯誤: {e}")
        finally:
            self.stop_system()
    
    def stop_system(self):
        """停止系統"""
        print("正在停止系統...")
        self.running = False
        self.continuous_rotation = False
        self.stop_motors()
    
    def cleanup(self):
        """清理資源"""
        print("正在清理資源...")
        
        # 停止馬達
        if hasattr(self, 'left_motor') and hasattr(self, 'right_motor'):
            self.stop_motors()
        
        # 關閉攝影機
        if self.camera_left:
            self.camera_left.release()
        if self.camera_right:
            self.camera_right.release()
        
        # 關閉OpenCV視窗
        cv2.destroyAllWindows()
        
        print("資源清理完成")

def main():
    """主函數"""
    print("=== Jetson Nano 簡化版障礙物檢測 ===")
    print("車子將持續原地轉動，直到偵測到障礙物")
    print()
    
    # 詢問是否啟用調試模式
    debug_mode = False
    try:
        response = input("是否啟用調試模式？(顯示檢測視窗) [y/N]: ").strip().lower()
        debug_mode = response.startswith('y')
    except:
        pass
    
    try:
        # 創建檢測系統
        detector = SimpleObstacleDetection()
        
        # 啟動系統
        detector.run(debug_mode=debug_mode)
        
    except Exception as e:
        print(f"系統啟動失敗: {e}")
        print("\n請檢查：")
        print("1. Motor HAT 是否正確連接")
        print("2. 攝影機是否正確連接")
        print("3. I2C 是否正常工作")

if __name__ == "__main__":
    main()