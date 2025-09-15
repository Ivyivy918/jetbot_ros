#!/usr/bin/env python3
"""
立體視覺模組 - 專為6公分間距的雙攝影機設計
提供深度感知和障礙物檢測功能
"""

import cv2
import numpy as np
import threading
import time
import pickle
import os

class StereoVision:
    def __init__(self):
        """
        初始化立體視覺系統
        攝影機間距: 6公分
        """
        # 攝影機配置
        self.config = {
            'width': 640,
            'height': 480,
            'framerate': 30,
            'baseline': 60,  # 6公分 = 60毫米
            'focal_length': 400,  # 預估焦距，需要校正
        }
        
        # 攝影機物件
        self.camera_left = None   # CSI-0
        self.camera_right = None  # CSI-1
        
        # 影像緩衝
        self.frame_left = None
        self.frame_right = None
        self.frame_lock = threading.Lock()
        
        # 執行狀態
        self.running = False
        self.capture_thread = None
        
        # 立體視覺參數
        self.stereo_params = {
            'num_disparities': 64,  # 必須是16的倍數
            'block_size': 15,       # 奇數，建議5-21
            'min_disparity': 0,
            'uniqueness_ratio': 10,
            'speckle_window_size': 100,
            'speckle_range': 32,
            'disp12_max_diff': 1
        }
        
        # 校正參數 (需要校正後填入)
        self.calibration_data = {
            'camera_matrix_left': None,
            'camera_matrix_right': None,
            'dist_coeffs_left': None,
            'dist_coeffs_right': None,
            'rotation_matrix': None,
            'translation_vector': None,
            'essential_matrix': None,
            'fundamental_matrix': None,
            'is_calibrated': False
        }
        
        # 障礙物檢測參數
        self.obstacle_params = {
            'safe_distance_mm': 500,    # 安全距離 50公分
            'warning_distance_mm': 300, # 警告距離 30公分
            'critical_distance_mm': 150, # 危險距離 15公分
            'roi_bottom_ratio': 0.7,    # 檢測區域從底部70%開始
        }
        
        # 初始化立體匹配器
        self.stereo_matcher = None
        self._init_stereo_matcher()
        
        # 初始化攝影機
        self.init_cameras()
        
        # 嘗試載入校正數據
        self.load_calibration()
    
    def init_cameras(self):
        """初始化雙攝影機"""
        print("初始化立體攝影機系統...")
        
        # 左攝影機 (CSI-0)
        try:
            gst_pipeline_left = self._create_gst_pipeline(0)
            self.camera_left = cv2.VideoCapture(gst_pipeline_left, cv2.CAP_GSTREAMER)
            
            if self.camera_left.isOpened():
                print("✓ 左攝影機 (CSI-0) 初始化成功")
            else:
                print("✗ 左攝影機初始化失敗")
                return False
        except Exception as e:
            print(f"✗ 左攝影機錯誤: {e}")
            return False
        
        # 右攝影機 (CSI-1)
        try:
            gst_pipeline_right = self._create_gst_pipeline(1)
            self.camera_right = cv2.VideoCapture(gst_pipeline_right, cv2.CAP_GSTREAMER)
            
            if self.camera_right.isOpened():
                print("✓ 右攝影機 (CSI-1) 初始化成功")
            else:
                print("✗ 右攝影機初始化失敗")
                return False
        except Exception as e:
            print(f"✗ 右攝影機錯誤: {e}")
            return False
        
        return True
    
    def _create_gst_pipeline(self, sensor_id):
        """創建 GStreamer 管道"""
        return (
            f"nvarguscamerasrc sensor_id={sensor_id} ! "
            f"video/x-raw(memory:NVMM), width={self.config['width']}, "
            f"height={self.config['height']}, format=NV12, "
            f"framerate={self.config['framerate']}/1 ! "
            "nvvidconv ! video/x-raw, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink drop=1 max-buffers=2"
        )
    
    def _init_stereo_matcher(self):
        """初始化立體匹配器"""
        # 使用 StereoBM 算法 (較快但精度較低)
        self.stereo_matcher = cv2.StereoBM_create(
            numDisparities=self.stereo_params['num_disparities'],
            blockSize=self.stereo_params['block_size']
        )
        
        # 設定進階參數
        self.stereo_matcher.setMinDisparity(self.stereo_params['min_disparity'])
        self.stereo_matcher.setUniquenessRatio(self.stereo_params['uniqueness_ratio'])
        self.stereo_matcher.setSpeckleWindowSize(self.stereo_params['speckle_window_size'])
        self.stereo_matcher.setSpeckleRange(self.stereo_params['speckle_range'])
        self.stereo_matcher.setDisp12MaxDiff(self.stereo_params['disp12_max_diff'])
    
    def start_capture(self):
        """開始立體攝影機擷取"""
        if not (self.camera_left and self.camera_right):
            print("錯誤: 立體攝影機未正確初始化")
            return False
        
        self.running = True
        self.capture_thread = threading.Thread(target=self._capture_loop)
        self.capture_thread.daemon = True
        self.capture_thread.start()
        
        print("立體攝影機擷取已開始")
        return True
    
    def stop_capture(self):
        """停止攝影機擷取"""
        self.running = False
        if self.capture_thread:
            self.capture_thread.join(timeout=2)
        print("立體攝影機擷取已停止")
    
    def _capture_loop(self):
        """攝影機擷取迴圈"""
        while self.running:
            with self.frame_lock:
                # 同時擷取兩個攝影機
                ret_left = ret_right = False
                
                if self.camera_left.isOpened():
                    ret_left, frame_left = self.camera_left.read()
                
                if self.camera_right.isOpened():
                    ret_right, frame_right = self.camera_right.read()
                
                # 只有當兩個攝影機都成功擷取時才更新
                if ret_left and ret_right:
                    self.frame_left = frame_left
                    self.frame_right = frame_right
            
            time.sleep(0.033)  # ~30 FPS
    
    def get_stereo_frames(self):
        """獲取同步的立體影像對"""
        with self.frame_lock:
            if self.frame_left is not None and self.frame_right is not None:
                return self.frame_left.copy(), self.frame_right.copy()
            return None, None
    
    def compute_disparity(self, left_frame=None, right_frame=None):
        """
        計算視差圖
        
        Returns:
            numpy.ndarray: 視差圖
        """
        if left_frame is None or right_frame is None:
            left_frame, right_frame = self.get_stereo_frames()
        
        if left_frame is None or right_frame is None:
            return None
        
        # 轉換為灰階
        gray_left = cv2.cvtColor(left_frame, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right_frame, cv2.COLOR_BGR2GRAY)
        
        # 如果已校正，使用校正後的影像
        if self.calibration_data['is_calibrated']:
            gray_left, gray_right = self._rectify_images(gray_left, gray_right)
        
        # 計算視差
        disparity = self.stereo_matcher.compute(gray_left, gray_right)
        
        # 轉換為浮點數並正規化
        disparity = disparity.astype(np.float32) / 16.0
        
        return disparity
    
    def disparity_to_depth(self, disparity):
        """
        將視差轉換為深度 (毫米)
        
        Args:
            disparity: 視差圖
            
        Returns:
            numpy.ndarray: 深度圖 (毫米)
        """
        # 避免除零錯誤
        disparity[disparity <= 0] = 0.1
        
        # 深度 = (基線 × 焦距) / 視差
        depth = (self.config['baseline'] * self.config['focal_length']) / disparity
        
        return depth
    
    def detect_obstacles_stereo(self):
        """
        使用立體視覺檢測障礙物
        
        Returns:
            dict: 障礙物檢測結果
        """
        # 獲取視差圖
        disparity = self.compute_disparity()
        
        if disparity is None:
            return {
                'left': False,
                'center': False,
                'right': False,
                'distances': {'left': float('inf'), 'center': float('inf'), 'right': float('inf')},
                'valid': False
            }
        
        # 轉換為深度
        depth_map = self.disparity_to_depth(disparity)
        
        # 分析三個區域
        return self._analyze_depth_regions(depth_map, disparity)
    
    def _analyze_depth_regions(self, depth_map, disparity):
        """分析深度圖的三個區域"""
        height, width = depth_map.shape
        
        # 定義檢測區域 (只檢測下方區域，避免天空干擾)
        roi_start_y = int(height * self.obstacle_params['roi_bottom_ratio'])
        roi_height = height - roi_start_y
        
        # 分割為左、中、右三個區域
        left_region = depth_map[roi_start_y:height, 0:width//3]
        center_region = depth_map[roi_start_y:height, width//3:2*width//3]
        right_region = depth_map[roi_start_y:height, 2*width//3:width]
        
        # 計算每個區域的最小距離 (最近的障礙物)
        def get_region_min_distance(region):
            # 過濾無效值
            valid_depths = region[(region > 50) & (region < 5000)]  # 5公分到5公尺
            
            if len(valid_depths) == 0:
                return float('inf')
            
            # 取下四分位數作為代表距離 (避免雜訊影響)
            return np.percentile(valid_depths, 25)
        
        left_distance = get_region_min_distance(left_region)
        center_distance = get_region_min_distance(center_region)
        right_distance = get_region_min_distance(right_region)
        
        # 判斷是否有障礙物
        safe_dist = self.obstacle_params['safe_distance_mm']
        
        result = {
            'left': left_distance < safe_dist,
            'center': center_distance < safe_dist,
            'right': right_distance < safe_dist,
            'distances': {
                'left': left_distance,
                'center': center_distance,
                'right': right_distance
            },
            'danger_level': self._assess_danger_level(center_distance),
            'disparity_map': disparity,
            'depth_map': depth_map,
            'valid': True
        }
        
        return result
    
    def _assess_danger_level(self, center_distance):
        """評估危險等級"""
        if center_distance < self.obstacle_params['critical_distance_mm']:
            return 'critical'
        elif center_distance < self.obstacle_params['warning_distance_mm']:
            return 'warning'
        elif center_distance < self.obstacle_params['safe_distance_mm']:
            return 'caution'
        else:
            return 'safe'
    
    def calibrate_cameras(self, calibration_images_path="./calibration_images"):
        """
        相機校正功能
        需要棋盤格標定圖片
        """
        print("開始攝影機校正...")
        
        # 棋盤格參數
        chessboard_size = (9, 6)  # 內角點數量
        square_size = 25  # 方格大小 (毫米)
        
        # 準備物體點
        objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
        objp *= square_size
        
        # 儲存點
        objpoints = []  # 3D點
        imgpoints_left = []  # 2D點 - 左攝影機
        imgpoints_right = []  # 2D點 - 右攝影機
        
        # 擷取校正影像
        print("請移動棋盤格到不同位置，按空白鍵擷取影像，按 'q' 完成擷取")
        
        capture_count = 0
        while capture_count < 20:  # 至少需要15-20張圖片
            left_frame, right_frame = self.get_stereo_frames()
            
            if left_frame is None or right_frame is None:
                continue
            
            # 轉換為灰階
            gray_left = cv2.cvtColor(left_frame, cv2.COLOR_BGR2GRAY)
            gray_right = cv2.cvtColor(right_frame, cv2.COLOR_BGR2GRAY)
            
            # 尋找棋盤格角點
            ret_left, corners_left = cv2.findChessboardCorners(gray_left, chessboard_size, None)
            ret_right, corners_right = cv2.findChessboardCorners(gray_right, chessboard_size, None)
            
            # 顯示影像
            display_left = left_frame.copy()
            display_right = right_frame.copy()
            
            if ret_left:
                cv2.drawChessboardCorners(display_left, chessboard_size, corners_left, ret_left)
            if ret_right:
                cv2.drawChessboardCorners(display_right, chessboard_size, corners_right, ret_right)
            
            combined = np.hstack([display_left, display_right])
            cv2.putText(combined, f"Captured: {capture_count}/20", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow('Stereo Calibration', combined)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord(' ') and ret_left and ret_right:
                # 擷取成功的影像
                objpoints.append(objp)
                imgpoints_left.append(corners_left)
                imgpoints_right.append(corners_right)
                capture_count += 1
                print(f"已擷取 {capture_count} 張校正影像")
            
            elif key == ord('q'):
                break
        
        cv2.destroyAllWindows()
        
        if capture_count < 10:
            print("校正影像數量不足，需要至少10張")
            return False
        
        # 執行校正
        print("正在進行攝影機校正...")
        
        image_size = (self.config['width'], self.config['height'])
        
        # 單眼校正
        ret_left, mtx_left, dist_left, rvecs_left, tvecs_left = cv2.calibrateCamera(
            objpoints, imgpoints_left, image_size, None, None)
        
        ret_right, mtx_right, dist_right, rvecs_right, tvecs_right = cv2.calibrateCamera(
            objpoints, imgpoints_right, image_size, None, None)
        
        # 立體校正
        ret_stereo, mtx_left, dist_left, mtx_right, dist_right, R, T, E, F = cv2.stereoCalibrate(
            objpoints, imgpoints_left, imgpoints_right,
            mtx_left, dist_left, mtx_right, dist_right, image_size,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
            flags=cv2.CALIB_FIX_INTRINSIC)
        
        if ret_stereo:
            # 儲存校正數據
            self.calibration_data = {
                'camera_matrix_left': mtx_left,
                'camera_matrix_right': mtx_right,
                'dist_coeffs_left': dist_left,
                'dist_coeffs_right': dist_right,
                'rotation_matrix': R,
                'translation_vector': T,
                'essential_matrix': E,
                'fundamental_matrix': F,
                'is_calibrated': True
            }
            
            # 更新焦距
            self.config['focal_length'] = float(mtx_left[0, 0])
            
            print("攝影機校正完成!")
            print(f"重投影誤差: {ret_stereo}")
            
            # 儲存校正數據
            self.save_calibration()
            return True
        else:
            print("攝影機校正失敗")
            return False
    
    def save_calibration(self, filename="stereo_calibration.pkl"):
        """儲存校正數據"""
        try:
            with open(filename, 'wb') as f:
                pickle.dump(self.calibration_data, f)
            print(f"校正數據已儲存到 {filename}")
        except Exception as e:
            print(f"儲存校正數據失敗: {e}")
    
    def load_calibration(self, filename="stereo_calibration.pkl"):
        """載入校正數據"""
        try:
            if os.path.exists(filename):
                with open(filename, 'rb') as f:
                    self.calibration_data = pickle.load(f)
                print(f"已載入校正數據: {filename}")
                
                # 更新焦距
                if self.calibration_data['camera_matrix_left'] is not None:
                    self.config['focal_length'] = float(
                        self.calibration_data['camera_matrix_left'][0, 0])
                
                return True
        except Exception as e:
            print(f"載入校正數據失敗: {e}")
        
        return False
    
    def create_debug_visualization(self, obstacle_result):
        """創建調試視覺化"""
        left_frame, right_frame = self.get_stereo_frames()
        
        if left_frame is None or right_frame is None:
            return None
        
        if not obstacle_result['valid']:
            return np.hstack([left_frame, right_frame])
        
        # 獲取深度圖和視差圖
        depth_map = obstacle_result['depth_map']
        disparity_map = obstacle_result['disparity_map']
        
        # 正規化視差圖用於顯示
        disp_vis = cv2.normalize(disparity_map, None, 0, 255, cv2.NORM_MINMAX)
        disp_vis = cv2.applyColorMap(disp_vis.astype(np.uint8), cv2.COLORMAP_JET)
        
        # 在左影像上繪製檢測區域和資訊
        debug_left = left_frame.copy()
        height, width = debug_left.shape[:2]
        
        # 繪製檢測區域
        roi_y = int(height * self.obstacle_params['roi_bottom_ratio'])
        
        # 左區域
        color = (0, 0, 255) if obstacle_result['left'] else (0, 255, 0)
        cv2.rectangle(debug_left, (0, roi_y), (width//3, height), color, 2)
        
        # 中區域
        color = (0, 0, 255) if obstacle_result['center'] else (0, 255, 0)
        cv2.rectangle(debug_left, (width//3, roi_y), (2*width//3, height), color, 2)
        
        # 右區域
        color = (0, 0, 255) if obstacle_result['right'] else (0, 255, 0)
        cv2.rectangle(debug_left, (2*width//3, roi_y), (width, height), color, 2)
        
        # 顯示距離資訊
        distances = obstacle_result['distances']
        y_offset = 30
        
        cv2.putText(debug_left, f"L: {distances['left']:.0f}mm", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(debug_left, f"C: {distances['center']:.0f}mm", (10, y_offset + 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(debug_left, f"R: {distances['right']:.0f}mm", (10, y_offset + 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # 危險等級
        danger_level = obstacle_result['danger_level']
        color_map = {
            'safe': (0, 255, 0),
            'caution': (0, 255, 255),
            'warning': (0, 165, 255),
            'critical': (0, 0, 255)
        }
        cv2.putText(debug_left, f"Status: {danger_level.upper()}", (10, y_offset + 75),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_map[danger_level], 2)
        
        # 組合顯示
        top_row = np.hstack([debug_left, right_frame])
        bottom_row = np.hstack([disp_vis, cv2.resize(disp_vis, (width, height))])
        
        return np.vstack([top_row, bottom_row])
    
    def cleanup(self):
        """清理資源"""
        print("正在清理立體視覺資源...")
        self.stop_capture()
        
        if self.camera_left:
            self.camera_left.release()
        if self.camera_right:
            self.camera_right.release()
        
        cv2.destroyAllWindows()
        print("立體視覺資源清理完成")