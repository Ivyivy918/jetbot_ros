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

        self.load_calibration()       # 載入標定參數 + 建立 rectify map
        self.setup_publishers()

        os.environ['DISPLAY'] = ':0'

        self.get_logger().info("Opening left camera (sensor-id=1)...")
        self.cap_left = cv2.VideoCapture(self.gst_pipeline(1), cv2.CAP_GSTREAMER)
        time.sleep(2.0)

        self.get_logger().info("Opening right camera (sensor-id=0)...")
        self.cap_right = cv2.VideoCapture(self.gst_pipeline(0), cv2.CAP_GSTREAMER)

        if not self.cap_left.isOpened():
            raise RuntimeError("無法開啟左相機 (sensor-id=1)")
        if not self.cap_right.isOpened():
            raise RuntimeError("無法開啟右相機 (sensor-id=0)")

        self.get_logger().info("✅ Stereo CSI cameras opened successfully")

        self.thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.thread.start()
        self.create_timer(1.0 / 15.0, self.publish_images)

    # ──────────────────────────────────────────────────────────
    def gst_pipeline(self, sensor_id):
        return (
            f"nvarguscamerasrc sensor-id={sensor_id} ! "
            f"video/x-raw(memory:NVMM),width=1280,height=720,framerate=60/1 ! "
            f"nvvidconv ! "
            f"video/x-raw,width=640,height=480,format=BGRx ! "
            f"videoconvert ! "
            f"video/x-raw,format=BGR ! "
            f"appsink drop=true max-buffers=1 sync=false"
        )

    # ──────────────────────────────────────────────────────────
    def load_calibration(self):
        pkg = get_package_share_directory('jetbot_ros')
        left_yaml  = os.path.join(pkg, 'config', 'left.yaml')
        right_yaml = os.path.join(pkg, 'config', 'right.yaml')

        with open(left_yaml,  'r') as f: left  = yaml.safe_load(f)
        with open(right_yaml, 'r') as f: right = yaml.safe_load(f)

        w = left['image_width']
        h = left['image_height']

        # ── 左相機內參 ──
        K_l = np.array(left['camera_matrix']['data'],
                       dtype=np.float64).reshape(3, 3)
        D_l = np.array(left['distortion_coefficients']['data'],
                       dtype=np.float64)
        R_l = np.array(left['rectification_matrix']['data'],
                       dtype=np.float64).reshape(3, 3)
        P_l = np.array(left['projection_matrix']['data'],
                       dtype=np.float64).reshape(3, 4)

        # ── 右相機內參 ──
        K_r = np.array(right['camera_matrix']['data'],
                       dtype=np.float64).reshape(3, 3)
        D_r = np.array(right['distortion_coefficients']['data'],
                       dtype=np.float64)
        R_r = np.array(right['rectification_matrix']['data'],
                       dtype=np.float64).reshape(3, 3)
        P_r = np.array(right['projection_matrix']['data'],
                       dtype=np.float64).reshape(3, 4)

        # ── 建立 rectification map（只算一次，之後 remap 很快）──
        self.map_l1, self.map_l2 = cv2.initUndistortRectifyMap(
            K_l, D_l, R_l, P_l, (w, h), cv2.CV_16SC2)
        self.map_r1, self.map_r2 = cv2.initUndistortRectifyMap(
            K_r, D_r, R_r, P_r, (w, h), cv2.CV_16SC2)

        # ── focal length & baseline（供 depth node 參考）──
        self.fx       = P_l[0, 0]
        self.baseline = abs(P_r[0, 3]) / self.fx   # baseline = |Tx| / fx

        # ── CameraInfo 訊息（使用 projection matrix 的參數）──
        self.left_info  = self.make_camera_info(left,  'camera_left_optical_frame')
        self.right_info = self.make_camera_info(right, 'camera_right_optical_frame')

        self.get_logger().info(
            f"📐 Calibration loaded | fx={self.fx:.2f}, baseline={self.baseline:.4f}m"
        )

    # ──────────────────────────────────────────────────────────
    def make_camera_info(self, calib, frame_id):
        msg = CameraInfo()
        msg.header.frame_id    = frame_id
        msg.width              = calib['image_width']
        msg.height             = calib['image_height']
        msg.distortion_model   = calib['distortion_model']
        msg.k = calib['camera_matrix']['data']
        msg.d = calib['distortion_coefficients']['data']
        msg.r = calib['rectification_matrix']['data']
        msg.p = calib['projection_matrix']['data']
        return msg

    # ──────────────────────────────────────────────────────────
    def setup_publishers(self):
        # raw：原始未校正影像（供標定用）
        self.pub_left_raw  = self.create_publisher(Image, '/camera_left/image_raw',  10)
        self.pub_right_raw = self.create_publisher(Image, '/camera_right/image_raw', 10)
        # rect：校正後影像（供深度估測、RTAB-Map 用）
        self.pub_left_rect  = self.create_publisher(Image, '/camera_left/image_rect',  10)
        self.pub_right_rect = self.create_publisher(Image, '/camera_right/image_rect', 10)
        # camera info
        self.pub_left_info  = self.create_publisher(CameraInfo, '/camera_left/camera_info',  10)
        self.pub_right_info = self.create_publisher(CameraInfo, '/camera_right/camera_info', 10)

    # ──────────────────────────────────────────────────────────
    def capture_loop(self):
        consecutive_failures = 0
        while self.running:
            ret_l, l = self.cap_left.read()
            ret_r, r = self.cap_right.read()
            if ret_l and ret_r:
                with self.lock:
                    self.left_img  = l.copy()
                    self.right_img = r.copy()
                consecutive_failures = 0
            else:
                consecutive_failures += 1
                if consecutive_failures % 30 == 1:
                    self.get_logger().warn(
                        f"⚠️ 相機讀取失敗 {consecutive_failures} 次 "
                        f"(left={ret_l}, right={ret_r})"
                    )
            time.sleep(0.005)

    # ──────────────────────────────────────────────────────────
    def publish_images(self):
        with self.lock:
            if self.left_img is None or self.right_img is None:
                return
            left_raw  = self.left_img.copy()
            right_raw = self.right_img.copy()

        stamp = self.get_clock().now().to_msg()

        # ── 原始影像 ──────────────────────────────────────────
        left_raw_msg  = self.bridge.cv2_to_imgmsg(left_raw,  encoding='bgr8')
        right_raw_msg = self.bridge.cv2_to_imgmsg(right_raw, encoding='bgr8')
        left_raw_msg.header.stamp  = stamp
        right_raw_msg.header.stamp = stamp
        left_raw_msg.header.frame_id  = 'camera_left_optical_frame'
        right_raw_msg.header.frame_id = 'camera_right_optical_frame'

        # ── Rectification（畸變校正 + 水平對齊）─────────────
        left_rect  = cv2.remap(left_raw,  self.map_l1, self.map_l2, cv2.INTER_LINEAR)
        right_rect = cv2.remap(right_raw, self.map_r1, self.map_r2, cv2.INTER_LINEAR)

        left_rect_msg  = self.bridge.cv2_to_imgmsg(left_rect,  encoding='bgr8')
        right_rect_msg = self.bridge.cv2_to_imgmsg(right_rect, encoding='bgr8')
        left_rect_msg.header.stamp  = stamp
        right_rect_msg.header.stamp = stamp
        left_rect_msg.header.frame_id  = 'camera_left_optical_frame'
        right_rect_msg.header.frame_id = 'camera_right_optical_frame'

        # ── camera_info ──────────────────────────────────────
        self.left_info.header.stamp  = stamp
        self.right_info.header.stamp = stamp

        # ── 發布 ─────────────────────────────────────────────
        self.pub_left_raw.publish(left_raw_msg)
        self.pub_right_raw.publish(right_raw_msg)
        self.pub_left_rect.publish(left_rect_msg)
        self.pub_right_rect.publish(right_rect_msg)
        self.pub_left_info.publish(self.left_info)
        self.pub_right_info.publish(self.right_info)

    # ──────────────────────────────────────────────────────────
    def destroy_node(self):
        self.running = False
        time.sleep(0.1)
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()