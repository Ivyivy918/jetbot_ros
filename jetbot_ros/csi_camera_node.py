#!/usr/bin/env python3
"""
CSI Camera Node for Jetson Orin Nano with JetPack 6.x
支援 IMX219 立體相機，使用 NVIDIA Argus 或 V4L2
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import yaml
import os
import subprocess

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
        self.declare_parameter('flip_method', 0)  # 0=none, 2=rotate-180

        sensor_id = self.get_parameter('sensor_id').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('camera_frame_id').value
        camera_info_url = self.get_parameter('camera_info_url').value
        self.output_width = self.get_parameter('output_width').value
        self.output_height = self.get_parameter('output_height').value
        flip_method = self.get_parameter('flip_method').value

        self.get_logger().info("="*60)
        self.get_logger().info("CSI Camera Node - Jetson Orin Nano JetPack 6.x")
        self.get_logger().info(f"Sensor ID: {sensor_id}")
        self.get_logger().info(f"Input: {width}x{height} @ {fps}fps")
        self.get_logger().info(f"Output: {self.output_width}x{self.output_height}")
        self.get_logger().info("="*60)

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

        # 嘗試多種方法開啟相機
        self.cap = None
        self.use_resize = False
        self.pipeline_type = None

        # 方法1: 嘗試 NVIDIA Argus (最佳選擇)
        if self.try_argus_camera(sensor_id, width, height, fps, flip_method):
            self.pipeline_type = "argus"
            self.get_logger().info("✓ Successfully using NVIDIA Argus camera pipeline")

        # 方法2: 嘗試 V4L2 with Bayer format
        elif self.try_v4l2_bayer_camera(sensor_id, width, height, fps):
            self.pipeline_type = "v4l2_bayer"
            self.get_logger().info("✓ Successfully using V4L2 Bayer pipeline")

        # 方法3: 嘗試 V4L2 簡單模式
        elif self.try_v4l2_simple_camera(sensor_id, width, height, fps):
            self.pipeline_type = "v4l2_simple"
            self.get_logger().info("✓ Successfully using V4L2 simple pipeline")

        # 方法4: 回退到 OpenCV
        elif self.try_opencv_camera(sensor_id, width, height, fps):
            self.pipeline_type = "opencv"
            self.use_resize = True
            self.get_logger().info("✓ Successfully using OpenCV VideoCapture")

        else:
            self.get_logger().error(f"✗ Failed to open camera with sensor_id={sensor_id}")
            self.get_logger().error("Please check:")
            self.get_logger().error("  1. Camera is properly connected")
            self.get_logger().error("  2. /dev/video device exists")
            self.get_logger().error("  3. GStreamer is installed")
            self.get_logger().error("  4. Permissions are correct")
            return

        # 定時器
        self.timer = self.create_timer(1.0/fps, self.timer_callback)
        self.frame_count = 0

    def try_argus_camera(self, sensor_id, width, height, fps, flip_method):
        """嘗試使用 NVIDIA Argus 相機 (推薦用於 Jetson)"""

        # 檢查是否有 nvarguscamerasrc
        try:
            result = subprocess.run(['which', 'nvarguscamerasrc'],
                                  capture_output=True, timeout=2)
            if result.returncode != 0:
                self.get_logger().info("nvarguscamerasrc not found, skipping Argus")
                return False
        except:
            return False

        self.get_logger().info("Trying NVIDIA Argus camera pipeline...")

        # Argus pipeline - 專為 Jetson 優化
        gst_pipeline = (
            f'nvarguscamerasrc sensor-id={sensor_id} ! '
            f'video/x-raw(memory:NVMM), width={width}, height={height}, '
            f'format=NV12, framerate={fps}/1 ! '
            f'nvvidconv flip-method={flip_method} ! '
            f'video/x-raw, width={self.output_width}, height={self.output_height}, format=BGRx ! '
            f'videoconvert ! '
            f'video/x-raw, format=BGR ! '
            f'appsink max-buffers=1 drop=true'
        )

        self.get_logger().info(f"Pipeline: {gst_pipeline}")

        try:
            self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
            if self.cap.isOpened():
                # 測試讀取一幀
                ret, frame = self.cap.read()
                if ret and frame is not None and frame.size > 0:
                    self.get_logger().info(f"Frame test OK: {frame.shape}")
                    return True
                else:
                    self.get_logger().warn("Argus pipeline opened but can't read frames")
                    self.cap.release()
                    self.cap = None
                    return False
            else:
                self.cap = None
                return False
        except Exception as e:
            self.get_logger().warn(f"Argus failed: {e}")
            if self.cap:
                self.cap.release()
                self.cap = None
            return False

    def try_v4l2_bayer_camera(self, sensor_id, width, height, fps):
        """嘗試使用 V4L2 with Bayer 格式"""

        video_device = f'/dev/video{sensor_id}'

        if not os.path.exists(video_device):
            self.get_logger().info(f"{video_device} not found, skipping V4L2")
            return False

        self.get_logger().info(f"Trying V4L2 Bayer pipeline on {video_device}...")

        # 嘗試不同的 Bayer 格式
        bayer_formats = ['rggb', 'grbg', 'bggr', 'gbrg']

        for bayer_format in bayer_formats:
            gst_pipeline = (
                f'v4l2src device={video_device} ! '
                f'video/x-bayer, width={width}, height={height}, '
                f'framerate={fps}/1, format={bayer_format} ! '
                f'bayer2rgb ! '
                f'videoscale ! '
                f'video/x-raw, width={self.output_width}, height={self.output_height} ! '
                f'videoconvert ! '
                f'video/x-raw, format=BGR ! '
                f'appsink max-buffers=1 drop=true'
            )

            try:
                self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
                if self.cap.isOpened():
                    # 測試讀取
                    ret, frame = self.cap.read()
                    if ret and frame is not None and frame.size > 0:
                        self.get_logger().info(f"V4L2 Bayer OK with format={bayer_format}")
                        self.get_logger().info(f"Pipeline: {gst_pipeline}")
                        return True
                    else:
                        self.cap.release()
                        self.cap = None
                else:
                    self.cap = None
            except Exception as e:
                if self.cap:
                    self.cap.release()
                    self.cap = None
                continue

        self.get_logger().info("V4L2 Bayer: All formats failed")
        return False

    def try_v4l2_simple_camera(self, sensor_id, width, height, fps):
        """嘗試使用 V4L2 簡單模式 (無 Bayer 轉換)"""

        video_device = f'/dev/video{sensor_id}'

        if not os.path.exists(video_device):
            return False

        self.get_logger().info(f"Trying V4L2 simple pipeline on {video_device}...")

        gst_pipeline = (
            f'v4l2src device={video_device} ! '
            f'video/x-raw, width={width}, height={height}, framerate={fps}/1 ! '
            f'videoscale ! '
            f'video/x-raw, width={self.output_width}, height={self.output_height} ! '
            f'videoconvert ! '
            f'video/x-raw, format=BGR ! '
            f'appsink max-buffers=1 drop=true'
        )

        self.get_logger().info(f"Pipeline: {gst_pipeline}")

        try:
            self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
            if self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret and frame is not None and frame.size > 0:
                    self.get_logger().info("V4L2 simple pipeline OK")
                    return True
                else:
                    self.cap.release()
                    self.cap = None
            else:
                self.cap = None
        except Exception as e:
            self.get_logger().warn(f"V4L2 simple failed: {e}")
            if self.cap:
                self.cap.release()
                self.cap = None

        return False

    def try_opencv_camera(self, sensor_id, width, height, fps):
        """回退到 OpenCV VideoCapture"""

        self.get_logger().info(f"Trying OpenCV VideoCapture with index {sensor_id}...")

        try:
            self.cap = cv2.VideoCapture(sensor_id)
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                self.cap.set(cv2.CAP_PROP_FPS, fps)

                # 測試讀取
                ret, frame = self.cap.read()
                if ret and frame is not None and frame.size > 0:
                    self.get_logger().info("OpenCV VideoCapture OK")
                    return True
                else:
                    self.cap.release()
                    self.cap = None
            else:
                self.cap = None
        except Exception as e:
            self.get_logger().warn(f"OpenCV failed: {e}")
            if self.cap:
                self.cap.release()
                self.cap = None

        return False

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
        """定時讀取並發佈相機畫面"""

        if not self.cap or not self.cap.isOpened():
            if self.frame_count % 30 == 0:
                self.get_logger().error("Camera is not opened!")
            self.frame_count += 1
            return

        ret, frame = self.cap.read()

        if not ret:
            if self.frame_count % 30 == 0:
                self.get_logger().warn(f"Failed to read frame (pipeline: {self.pipeline_type})")
            self.frame_count += 1
            return

        if frame is None or frame.size == 0:
            if self.frame_count % 30 == 0:
                self.get_logger().warn("Received empty frame")
            self.frame_count += 1
            return

        # 手動縮放（如果需要）
        if self.use_resize:
            if frame.shape[1] != self.output_width or frame.shape[0] != self.output_height:
                frame = cv2.resize(frame, (self.output_width, self.output_height))

        # 發佈影像 - 使用 bgr8 編碼
        now = self.get_clock().now().to_msg()

        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = now
            img_msg.header.frame_id = self.frame_id
            self.image_pub.publish(img_msg)

            # 發佈 camera_info
            self.camera_info.header.stamp = now
            self.info_pub.publish(self.camera_info)

            self.frame_count += 1

            if self.frame_count == 1:
                self.get_logger().info(f"✓ Started publishing frames ({frame.shape[1]}x{frame.shape[0]})")
            elif self.frame_count % 100 == 0:
                self.get_logger().info(f"Published {self.frame_count} frames")

        except Exception as e:
            self.get_logger().error(f"Error publishing frame: {e}")

    def __del__(self):
        if hasattr(self, 'cap') and self.cap is not None:
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
