#!/usr/bin/env python3
import rclpy
import cv2
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
        # Load calibration
        # -------------------------------
        self.load_calibration()

        # -------------------------------
        # Publishers (RTAB-Map compatible)
        # -------------------------------
        self.pub_left = self.create_publisher(Image, '/camera_left/image_rect', 10)
        self.pub_right = self.create_publisher(Image, '/camera_right/image_rect', 10)
        self.pub_left_info = self.create_publisher(CameraInfo, '/camera_left/camera_info', 10)
        self.pub_right_info = self.create_publisher(CameraInfo, '/camera_right/camera_info', 10)

        # -------------------------------
        # Init CSI cameras (灰階)
        # -------------------------------
        self.cap_left = cv2.VideoCapture(self.gst_pipeline(0), cv2.CAP_GSTREAMER)
        self.cap_right = cv2.VideoCapture(self.gst_pipeline(1), cv2.CAP_GSTREAMER)

        if not self.cap_left.isOpened() or not self.cap_right.isOpened():
            raise RuntimeError("❌ 無法開啟 CSI 相機")

        self.get_logger().info("✅ Stereo CSI cameras (灰階) 已開啟")

        self.thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.thread.start()

        # 發布定時器
        self.create_timer(1.0 / 15.0, self.publish_images)

    # -------------------------------
    def gst_pipeline(self, sensor_id):
        # 灰階格式 NVMM -> GRAY8
        return (
            f"nvarguscamerasrc sensor-id={sensor_id} ! "
            "video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
            "nvvidconv ! videoconvert ! video/x-raw, format=GRAY8 ! appsink"
        )

    # -------------------------------
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

        Tx = right['projection_matrix']['data'][3]
        self.baseline = -Tx / self.fx

        self.get_logger().info(f"📐 fx={self.fx:.2f}, baseline={self.baseline:.4f} m")

        self.left_info = self.make_camera_info(left, 'camera_left_optical_frame')
        self.right_info = self.make_camera_info(right, 'camera_right_optical_frame')

    # -------------------------------
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

    # -------------------------------
    def capture_loop(self):
        while self.running:
            ret_l, l = self.cap_left.read()
            ret_r, r = self.cap_right.read()
            if ret_l and ret_r:
                with self.lock:
                    self.left_img = l.copy()
                    self.right_img = r.copy()
            time.sleep(0.005)

    # -------------------------------
    def publish_images(self):
        with self.lock:
            if self.left_img is None or self.right_img is None:
                return
            left = self.left_img
            right = self.right_img

        stamp = self.get_clock().now().to_msg()

        # 灰階編碼
        left_msg = self.bridge.cv2_to_imgmsg(left, encoding='mono8')
        right_msg = self.bridge.cv2_to_imgmsg(right, encoding='mono8')

        left_msg.header.stamp = stamp
        right_msg.header.stamp = stamp
        self.left_info.header.stamp = stamp
        self.right_info.header.stamp = stamp

        self.pub_left.publish(left_msg)
        self.pub_right.publish(right_msg)
        self.pub_left_info.publish(self.left_info)
        self.pub_right_info.publish(self.right_info)

    # -------------------------------
    def destroy_node(self):
        self.running = False
        self.cap_left.release()
        self.cap_right.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
