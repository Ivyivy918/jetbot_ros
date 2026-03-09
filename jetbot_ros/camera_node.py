#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np
import yaml
import os
import threading
import time

from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory


class StereoCameraNode(Node):
    def __init__(self):
        super().__init__('stereo_camera_node')

        self.bridge = CvBridge()
        self.lock = threading.Lock()

        self.left_img = None
        self.right_img = None
        self.running = True

        # -------------------------------
        # 讀取相機校正檔 (確保你有把 left.yaml / right.yaml 放在 config 資料夾下)
        # -------------------------------
        self.load_calibration()

        # -------------------------------
        # 發佈者設定 (符合 RTAB-Map 與下游節點需求)
        # -------------------------------
        self.pub_left = self.create_publisher(Image, '/camera_left/image_rect', 10)
        self.pub_right = self.create_publisher(Image, '/camera_right/image_rect', 10)
        self.pub_left_info = self.create_publisher(CameraInfo, '/camera_left/camera_info', 10)
        self.pub_right_info = self.create_publisher(CameraInfo, '/camera_right/camera_info', 10)

        # -------------------------------
        # 初始化 CSI 相機 (使用優化後的 GStreamer 管線)
        # -------------------------------
        self.cap_left = cv2.VideoCapture(self.gst_pipeline(1), cv2.CAP_GSTREAMER)
        self.cap_right = cv2.VideoCapture(self.gst_pipeline(0), cv2.CAP_GSTREAMER)

        if not self.cap_left.isOpened() or not self.cap_right.isOpened():
            raise RuntimeError("❌ 無法開啟 CSI 相機，請檢查 nvargus-daemon 狀態")

        self.get_logger().info("✅ Stereo CSI cameras opened successfully (Headless Mode)")

        # 啟動背景擷取執行緒
        self.thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.thread.start()

        # 設定發佈頻率 (15 FPS 避免網路塞車)
        self.create_timer(1.0 / 15.0, self.publish_images)

    # -------------------------------------------------------
    # 優化版的 GStreamer 管線 (防崩潰、降延遲)
    # -------------------------------------------------------
    def gst_pipeline(self, sensor_id):
        return (
            f"nvarguscamerasrc sensor-id={sensor_id} ! "
            "video/x-raw(memory:NVMM), width=640, height=480, framerate=21/1 ! "
            "nvvidconv ! videoconvert ! video/x-raw, format=GRAY8 ! appsink drop=true max-buffers=1"
        )

    # -------------------------------------------------------
    def load_calibration(self):
        pkg = get_package_share_directory('jetbot_ros')
        left_yaml = os.path.join(pkg, 'config', 'left.yaml')
        right_yaml = os.path.join(pkg, 'config', 'right.yaml')

        with open(left_yaml, 'r') as f:
            left = yaml.safe_load(f)
        with open(right_yaml, 'r') as f:
            right = yaml.safe_load(f)

        self.fx = left['camera_matrix']['data'][0]
        self.fy = left['camera_matrix']['data'][4]
        self.cx = left['camera_matrix']['data'][2]
        self.cy = left['camera_matrix']['data'][5]

        # 計算基準線長度 (baseline)
        Tx = right['projection_matrix']['data'][3]
        self.baseline = -Tx / self.fx

        self.get_logger().info(
            f"📐 fx={self.fx:.2f}, baseline={self.baseline:.4f} m"
        )

        self.left_info = self.make_camera_info(left, 'camera_left_optical_frame')
        self.right_info = self.make_camera_info(right, 'camera_right_optical_frame')

    # -------------------------------------------------------
    def make_camera_info(self, calib, frame_id):
        msg = CameraInfo()
        msg.header.frame_id = frame_id
        msg.width = calib['image_width']
        msg.height = calib['image_height']
        msg.distortion_model = calib['distortion_model']
        msg.k = calib['camera_matrix']['data']
        msg.d = calib['distortion_coefficients']['data']
        msg.r = calib['rectification_matrix']['data']
        msg.p = calib['projection_matrix']['data']
        return msg

    # -------------------------------------------------------
    def capture_loop(self):
        while self.running:
            ret_l, l = self.cap_left.read()
            ret_r, r = self.cap_right.read()
            if ret_l and ret_r:
                with self.lock:
                    self.left_img = l.copy()
                    self.right_img = r.copy()
            # 微小休眠避免吃光 CPU
            time.sleep(0.005)

    # -------------------------------------------------------
    def publish_images(self):
        with self.lock:
            if self.left_img is None or self.right_img is None:
                return
            left = self.left_img
            right = self.right_img

        stamp = self.get_clock().now().to_msg()

        # 這裡發佈的是 mono8 (單色灰階)，是深度計算和 RTAB-Map 的最愛
        left_msg = self.bridge.cv2_to_imgmsg(left, encoding='bgr8')
        right_msg = self.bridge.cv2_to_imgmsg(right, encoding='bgr8')

        left_msg.header.stamp = stamp
        right_msg.header.stamp = stamp
        left_msg.header.frame_id = 'camera_left_optical_frame'
        right_msg.header.frame_id = 'camera_right_optical_frame'

        self.left_info.header.stamp = stamp
        self.right_info.header.stamp = stamp

        self.pub_left.publish(left_msg)
        self.pub_right.publish(right_msg)
        self.pub_left_info.publish(self.left_info)
        self.pub_right_info.publish(self.right_info)

    # -------------------------------------------------------
    def destroy_node(self):
        self.running = False
        self.cap_left.release()
        self.cap_right.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()