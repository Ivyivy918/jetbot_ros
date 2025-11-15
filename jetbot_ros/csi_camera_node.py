#!/usr/bin/env python3
"""
CSI Camera Node for Jetson Orin Nano with JetPack 6.x
支援立體相機，使用 V4L2 和 Bayer 格式
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import yaml
import os

class CSICameraNode(Node):
    def __init__(self):
        super().__init__('csi_camera_node')
        
        # 參數
        self.declare_parameter('sensor_id', 0)
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('camera_frame_id', 'camera_optical_frame')
        self.declare_parameter('camera_info_url', '')
        self.declare_parameter('output_width', 640)
        self.declare_parameter('output_height', 480)
        
        sensor_id = self.get_parameter('sensor_id').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('camera_frame_id').value
        camera_info_url = self.get_parameter('camera_info_url').value
        self.output_width = self.get_parameter('output_width').value
        self.output_height = self.get_parameter('output_height').value
        
        video_device = f'/dev/video{sensor_id}'
        
        self.get_logger().info(f"Jetson Orin Nano JetPack 6.x")
        self.get_logger().info(f"Using device: {video_device}")
        
        # GStreamer pipeline for Jetson Orin Nano
        # 使用 Bayer 格式轉 RGB，然後縮放到目標解析度
        gst_pipeline = (
            f'v4l2src device={video_device} ! '
            f'video/x-bayer, width={width}, height={height}, framerate={fps}/1, format=rggb ! '
            f'bayer2rgb ! '
            f'videoscale ! '
            f'video/x-raw, width={self.output_width}, height={self.output_height} ! '
            f'videoconvert ! '
            f'video/x-raw, format=BGR ! '
            f'appsink max-buffers=1 drop=true'
        )
        
        self.get_logger().info(f"Pipeline:\n{gst_pipeline}")
        
        # 開啟相機
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        self.use_resize = False
        
        if not self.cap.isOpened():
            self.get_logger().error("Failed with GStreamer, trying OpenCV...")
            
            # 備用：直接用 OpenCV
            self.cap = cv2.VideoCapture(sensor_id)
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                self.cap.set(cv2.CAP_PROP_FPS, fps)
                self.use_resize = True
                self.get_logger().info("Using OpenCV VideoCapture")
            else:
                self.get_logger().error(f"Failed to open camera {sensor_id}")
                return
        
        self.get_logger().info(f"Successfully opened camera sensor-id={sensor_id}")
        
        # 發佈器
        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'camera_info', 10)
        
        self.bridge = CvBridge()
        
        # 初始化 camera_info
        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = self.frame_id
        self.camera_info.width = self.output_width
        self.camera_info.height = self.output_height
        
        # 載入校正檔案
        self.load_camera_info(camera_info_url)
        
        # 定時器
        self.timer = self.create_timer(1.0/fps, self.timer_callback)
        self.frame_count = 0
    
    def load_camera_info(self, camera_info_url):
        """載入相機校正參數"""
        
        if not camera_info_url:
            self.get_logger().warn("No camera_info_url provided")
            return
        
        if not camera_info_url.startswith('file://'):
            self.get_logger().warn("camera_info_url must start with file://")
            return
        
        yaml_file = camera_info_url.replace('file://', '')
        
        if not os.path.exists(yaml_file):
            self.get_logger().warn(f"Calibration file not found: {yaml_file}")
            return
        
        try:
            with open(yaml_file, 'r') as f:
                calib_data = yaml.safe_load(f)
            
            # 取得原始影像尺寸
            orig_width = calib_data.get('image_width', 640)
            orig_height = calib_data.get('image_height', 480)
            
            self.get_logger().info(f"Calibration size: {orig_width}x{orig_height}")
            self.get_logger().info(f"Output size: {self.output_width}x{self.output_height}")
            
            # 檢查是否需要縮放
            if orig_width == self.output_width and orig_height == self.output_height:
                # 不需要縮放
                self.get_logger().info("Calibration matches output size - using directly")
                scale_x = 1.0
                scale_y = 1.0
            else:
                # 需要縮放
                scale_x = self.output_width / orig_width
                scale_y = self.output_height / orig_height
                self.get_logger().info(f"Scaling calibration: {scale_x:.3f}x{scale_y:.3f}")
            
            # 載入並縮放 camera_matrix
            k = list(calib_data['camera_matrix']['data'])
            k[0] *= scale_x  # fx
            k[2] *= scale_x  # cx
            k[4] *= scale_y  # fy
            k[5] *= scale_y  # cy
            self.camera_info.k = k
            
            # 載入 distortion_coefficients
            self.camera_info.d = list(calib_data['distortion_coefficients']['data'])
            
            # 載入 rectification_matrix
            self.camera_info.r = list(calib_data['rectification_matrix']['data'])
            
            # 載入並縮放 projection_matrix
            p = list(calib_data['projection_matrix']['data'])
            p[0] *= scale_x  # fx
            p[2] *= scale_x  # cx
            p[5] *= scale_y  # fy
            p[6] *= scale_y  # cy
            self.camera_info.p = p
            
            # 載入 distortion_model
            self.camera_info.distortion_model = calib_data.get('distortion_model', 'plumb_bob')
            
            self.get_logger().info(f"✓ Loaded camera calibration from {yaml_file}")
            self.get_logger().info(f"  fx={k[0]:.1f}, fy={k[4]:.1f}, cx={k[2]:.1f}, cy={k[5]:.1f}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load camera calibration: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if not ret:
            if self.frame_count % 30 == 0:
                self.get_logger().warn("Failed to read frame")
            self.frame_count += 1
            return
        
        # 如果需要手動縮放
        if self.use_resize:
            if frame.shape[1] != self.output_width or frame.shape[0] != self.output_height:
                frame = cv2.resize(frame, (self.output_width, self.output_height))
        
        # 轉換 BGR 到 RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # 發佈影像
        now = self.get_clock().now().to_msg()
        
        img_msg = self.bridge.cv2_to_imgmsg(frame_rgb, encoding='rgb8')
        img_msg.header.stamp = now
        img_msg.header.frame_id = self.frame_id
        self.image_pub.publish(img_msg)
        
        # 發佈 camera_info
        self.camera_info.header.stamp = now
        self.info_pub.publish(self.camera_info)
        
        self.frame_count += 1
        
        if self.frame_count == 1:
            self.get_logger().info("Started publishing frames")
        elif self.frame_count % 100 == 0:
            self.get_logger().info(f"Published {self.frame_count} frames")
    
    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = CSICameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()