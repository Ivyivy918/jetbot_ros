#!/usr/bin/env python3
import rclpy
import cv2
import threading
import time
import yaml
import os
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Image storage
        self.left_image = None
        self.right_image = None
        self.image_lock = threading.Lock()
        
        # Camera objects
        self.camera_left = None
        self.camera_right = None
        self.capture_thread = None
        self.running = False
        
        # Publishers - Images
        self.left_image_pub = self.create_publisher(
            Image, '/camera_left/image_raw', 10)
        self.right_image_pub = self.create_publisher(
            Image, '/camera_right/image_raw', 10)
        
        # Publishers - Camera Info
        self.left_info_pub = self.create_publisher(
            CameraInfo, '/camera_left/camera_info', 10)
        self.right_info_pub = self.create_publisher(
            CameraInfo, '/camera_right/camera_info', 10)
        
        # Load camera calibration
        self.left_camera_info = self.load_camera_info('left.yaml')
        self.right_camera_info = self.load_camera_info('right.yaml')
        
        if self.left_camera_info and self.right_camera_info:
            self.get_logger().info("Camera calibration loaded successfully")
        else:
            self.get_logger().warning("Failed to load camera calibration")
        
        # Initialize cameras
        self.init_physical_cameras()
    
    def load_camera_info(self, filename):
        """Load camera calibration from yaml file"""
        try:
            # 使用 ROS2 標準方式取得套件路徑
            pkg_share = get_package_share_directory('jetbot_ros')
            config_path = os.path.join(pkg_share, 'config', filename)
            
            # 如果找不到，嘗試開發環境路徑
            if not os.path.exists(config_path):
                config_path = os.path.expanduser(f'~/jetbot_ros/config/{filename}')
            
            if not os.path.exists(config_path):
                self.get_logger().error(f"Calibration file not found: {config_path}")
                return None
            
            self.get_logger().info(f"Loading calibration from: {config_path}")
            
            with open(config_path, 'r') as f:
                calib_data = yaml.safe_load(f)
            
            camera_info = CameraInfo()
            camera_info.width = calib_data['image_width']
            camera_info.height = calib_data['image_height']
            camera_info.distortion_model = calib_data['distortion_model']
            
            camera_info.k = calib_data['camera_matrix']['data']
            camera_info.d = calib_data['distortion_coefficients']['data']
            camera_info.r = calib_data['rectification_matrix']['data']
            camera_info.p = calib_data['projection_matrix']['data']
            
            # 設定正確的 frame_id（與 TF 一致）
            if 'left' in filename:
                camera_info.header.frame_id = 'camera_left_link'
            elif 'right' in filename:
                camera_info.header.frame_id = 'camera_right_link'
            
            return camera_info
            
        except Exception as e:
            self.get_logger().error(f"Failed to load camera info: {e}")
            return None
    
    def init_physical_cameras(self):
        """Initialize CSI cameras with GStreamer pipeline"""
        try:
            self.get_logger().info("Initializing dual CSI cameras...")
            
            # Left camera (CSI-0)
            gst_left = (
                "nvarguscamerasrc sensor_id=0 ! "
                "video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! "
                "nvvidconv ! video/x-raw, format=BGRx ! "
                "videoconvert ! video/x-raw, format=BGR ! "
                "appsink drop=1 max-buffers=2"
            )
            self.camera_left = cv2.VideoCapture(gst_left, cv2.CAP_GSTREAMER)
            
            # Right camera (CSI-1)
            gst_right = (
                "nvarguscamerasrc sensor_id=1 ! "
                "video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! "
                "nvvidconv ! video/x-raw, format=BGRx ! "
                "videoconvert ! video/x-raw, format=BGR ! "
                "appsink drop=1 max-buffers=2"
            )
            self.camera_right = cv2.VideoCapture(gst_right, cv2.CAP_GSTREAMER)
            
            # Check camera initialization
            if self.camera_left.isOpened() and self.camera_right.isOpened():
                self.get_logger().info("Dual CSI cameras initialized successfully")
                self.start_capture_thread()
            else:
                left_status = "OK" if self.camera_left.isOpened() else "FAILED"
                right_status = "OK" if self.camera_right.isOpened() else "FAILED"
                self.get_logger().error(f"Camera initialization - Left: {left_status}, Right: {right_status}")
                
        except Exception as e:
            self.get_logger().error(f"Camera initialization error: {e}")
    
    def start_capture_thread(self):
        """Start camera capture thread"""
        self.running = True
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()
        self.get_logger().info("Camera capture thread started")
    
    def capture_loop(self):
        """Main camera capture loop"""
        while self.running:
            try:
                frames_captured = False
                current_time = self.get_clock().now().to_msg()
                
                # Read left camera
                if self.camera_left and self.camera_left.isOpened():
                    ret_left, frame_left = self.camera_left.read()
                    if ret_left:
                        with self.image_lock:
                            self.left_image = frame_left
                            frames_captured = True
                        
                        # Publish left camera image and info
                        try:
                            img_msg = self.bridge.cv2_to_imgmsg(frame_left, 'bgr8')
                            img_msg.header.stamp = current_time
                            img_msg.header.frame_id = 'camera_left_link'
                            self.left_image_pub.publish(img_msg)
                            
                            if self.left_camera_info:
                                self.left_camera_info.header.stamp = current_time
                                self.left_camera_info.header.frame_id = 'camera_left_link'
                                self.left_info_pub.publish(self.left_camera_info)
                        except Exception as e:
                            self.get_logger().debug(f"Error publishing left camera: {e}")
                
                # Read right camera  
                if self.camera_right and self.camera_right.isOpened():
                    ret_right, frame_right = self.camera_right.read()
                    if ret_right:
                        with self.image_lock:
                            self.right_image = frame_right
                            frames_captured = True
                        
                        # Publish right camera image and info
                        try:
                            img_msg = self.bridge.cv2_to_imgmsg(frame_right, 'bgr8')
                            img_msg.header.stamp = current_time
                            img_msg.header.frame_id = 'camera_right_link'
                            self.right_image_pub.publish(img_msg)
                            
                            if self.right_camera_info:
                                self.right_camera_info.header.stamp = current_time
                                self.right_camera_info.header.frame_id = 'camera_right_link'
                                self.right_info_pub.publish(self.right_camera_info)
                        except Exception as e:
                            self.get_logger().debug(f"Error publishing right camera: {e}")
                
                if not frames_captured:
                    self.get_logger().warning("Failed to capture camera frames", 
                                            throttle_duration_sec=5.0)
                    
            except Exception as e:
                self.get_logger().error(f"Camera capture error: {e}")
                break
            
            time.sleep(0.033)  # ~30 FPS
    
    def cleanup(self):
        """Clean up resources"""
        self.running = False
        
        if self.capture_thread:
            self.capture_thread.join(timeout=1)
        
        if self.camera_left:
            self.camera_left.release()
        if self.camera_right:
            self.camera_right.release()
    
    def destroy_node(self):
        self.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CameraNode()
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\nShutdown signal received")
    except Exception as e:
        print(f"Execution error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()