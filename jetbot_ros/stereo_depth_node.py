#!/usr/bin/env python3
"""
StereoDepthNode — 雙目深度估測節點
輸出：
  /camera/depth/image_raw     : 深度圖（float32，單位公尺）
  /camera/depth/visual        : 視覺化深度圖（8bit，可在 RViz 顯示）
  /obstacle/min_distance      : 正前方最近距離（float32，單位公尺）
  /camera/disparity           : 視差圖（float32）
"""
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import yaml
import os
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import message_filters
from ament_index_python.packages import get_package_share_directory


class StereoDepthNode(Node):
    def __init__(self):
        super().__init__('stereo_depth_node')
        self.bridge = CvBridge()

        # ── 載入標定參數 ──────────────────────────────────────
        self.load_calibration()

        # ── 訂閱影像（用 image_rect，已做過鏡頭畸變校正）────────
        # 若 camera_node 只發布 image_raw，改成 /camera_left/image_raw
        left_sub  = message_filters.Subscriber(self, Image, '/camera_left/image_rect')
        right_sub = message_filters.Subscriber(self, Image, '/camera_right/image_rect')
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [left_sub, right_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.image_callback)

        # ── 發布器 ────────────────────────────────────────────
        self.pub_depth   = self.create_publisher(Image,   '/camera/depth/image_raw', 10)
        self.pub_visual  = self.create_publisher(Image,   '/camera/depth/visual',    10)
        self.pub_disp    = self.create_publisher(Image,   '/camera/disparity',       10)
        self.pub_mindist = self.create_publisher(Float32, '/obstacle/min_distance',  10)

        # ── StereoSGBM 參數（可透過 ROS parameter 調整）────────
        # Fix 1: num_disparities 從 64 調高至 336（必須是 16 的倍數）
        #   理由：baseline=0.1487m, fx_rect=1092.549
        #   50cm 物體需要視差 = 1092.549*0.1487/0.5 ≈ 325 pixels
        #   原本 64 只能偵測 >260cm 的物體，近物全部看不到
        self.declare_parameter('num_disparities', 544)
        self.declare_parameter('block_size',      15)
        self.declare_parameter('min_disparity',   0)

        # 避障用：只看影像中央幾 % 的區域
        self.declare_parameter('roi_center_ratio', 0.4)
        # 最近距離警告門檻（公尺）
        self.declare_parameter('min_dist_threshold', 0.5)

        # ── CLAHE 影像增強（參考 AlexJinlei/Stereo_Vision_Camera）────
        # 提升低對比度場景的視差匹配品質
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        self.build_stereo()
        self.get_logger().info(
            f'StereoDepthNode ready | '
            f'fx={self.fx:.1f}, baseline={self.baseline:.4f}m, '
            f'num_disparities={self.get_parameter("num_disparities").value}'
        )

    # ─────────────────────────────────────────────────────────
    def load_calibration(self):
        pkg = get_package_share_directory('jetbot_ros')
        left_yaml  = os.path.join(pkg, 'config', 'left.yaml')
        right_yaml = os.path.join(pkg, 'config', 'right.yaml')

        with open(left_yaml,  'r') as f: left  = yaml.safe_load(f)
        with open(right_yaml, 'r') as f: right = yaml.safe_load(f)

        # Fix 2: 訂閱 image_rect（已 rectify 的影像），fx 應來自 projection_matrix
        #   camera_matrix fx=647.197（原始影像）vs projection_matrix fx=1092.549（rectify 後）
        #   兩者雖然 fx*baseline 數值相同，但 baseline 顯示值會錯誤（0.25m vs 實際 0.15m）
        self.fx = left['projection_matrix']['data'][0]   # rectified fx，例：1092.549
        self.fy = left['projection_matrix']['data'][5]   # rectified fy

        # cx, cy 也從 projection_matrix 讀取（rectified 光學中心）
        self.cx = left['projection_matrix']['data'][2]
        self.cy = left['projection_matrix']['data'][6]

        # baseline = |Tx / fx_rect|（Tx 在右相機 projection matrix 的 [0,3]）
        Tx = right['projection_matrix']['data'][3]       # = -fx_rect * baseline
        self.baseline = abs(Tx / self.fx)                # 正確實體 baseline（公尺）

        # 影像尺寸
        self.img_w = left['image_width']
        self.img_h = left['image_height']

        self.get_logger().info(
            f'Calibration loaded | '
            f'fx={self.fx:.2f}, baseline={self.baseline:.4f}m, '
            f'size={self.img_w}x{self.img_h}'
        )

    # ─────────────────────────────────────────────────────────
    def build_stereo(self):
        nd  = self.get_parameter('num_disparities').value
        bs  = self.get_parameter('block_size').value
        md  = self.get_parameter('min_disparity').value

        self.stereo = cv2.StereoSGBM_create(
            minDisparity      = md,
            numDisparities    = nd,
            blockSize         = bs,
            P1                = 8  * 3 * bs ** 2,
            P2                = 32 * 3 * bs ** 2,
            disp12MaxDiff     = 1,
            uniquenessRatio   = 10,
            speckleWindowSize = 100,
            speckleRange      = 32,
            mode              = cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )

    # ─────────────────────────────────────────────────────────
    def image_callback(self, left_msg, right_msg):
        try:
            left_gray  = self.bridge.imgmsg_to_cv2(left_msg,  'mono8')
            right_gray = self.bridge.imgmsg_to_cv2(right_msg, 'mono8')

            # ── CLAHE 增強（改善低對比度場景的視差匹配）─────────
            left_gray  = self.clahe.apply(left_gray)
            right_gray = self.clahe.apply(right_gray)

            # ── 1. 計算視差圖 ──────────────────────────────────
            disp_raw = self.stereo.compute(
                left_gray, right_gray
            ).astype(np.float32) / 16.0   # SGBM 輸出固定要除 16

            # ── 2. 視差 → 深度（公尺）────────────────────────
            # depth = fx_rect * baseline / disparity
            # 視差 <= 0 的點（無效）設為 0
            valid = disp_raw > 0.0
            depth = np.zeros_like(disp_raw)
            depth[valid] = (self.fx * self.baseline) / disp_raw[valid]

            # Fix 4: 下限從 0.1m 降為 0.05m，避免 5~10cm 近物被誤刪
            # 上限 5m 已足夠室內避障使用
            depth[(depth < 0.50) | (depth > 5.0)] = 0.0

            # ── 3. 正前方最近距離（避障用）────────────────────
            roi_ratio = self.get_parameter('roi_center_ratio').value
            h, w = depth.shape
            y1 = int(h * (0.5 - roi_ratio / 2))
            y2 = int(h * (0.5 + roi_ratio / 2))
            x1 = int(w * (0.5 - roi_ratio / 2))
            x2 = int(w * (0.5 + roi_ratio / 2))
            roi = depth[y1:y2, x1:x2]
            valid_roi = roi[roi > 0.0]

            if valid_roi.size > 0:
                min_dist = float(np.percentile(valid_roi, 5))  # 5th percentile，濾掉雜訊
            else:
                min_dist = 0.0

            # Fix 3: 發布單位統一為公尺（移除原本的 *100 換算）
            # topic 說明已標注「單位公尺」，不應在這裡換算成公分
            dist_msg = Float32()
            dist_msg.data = min_dist * 100.0
            self.pub_mindist.publish(dist_msg)

            threshold = self.get_parameter('min_dist_threshold').value
            if 0 < min_dist < threshold:
                self.get_logger().warn(
                    f'障礙物距離 {min_dist * 100:.1f}cm（門檻 {threshold * 100:.0f}cm）',
                    throttle_duration_sec=1.0
                )

            # ── 4. 發布深度圖（float32，公尺）─────────────────
            depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='32FC1')
            depth_msg.header = left_msg.header
            self.pub_depth.publish(depth_msg)

            # ── 5. 視覺化深度圖（colormap，方便 RViz 顯示）─────
            mask = depth > 0
            if mask.any():
                d_norm = np.clip(depth, 0.05, 5.0)
                d_u8 = ((d_norm - 0.05) / 4.95 * 255).astype(np.uint8)
                depth_vis = cv2.applyColorMap(255 - d_u8, cv2.COLORMAP_JET)
                depth_vis[~mask] = 0

                # 在影像上標出 ROI 框和最近距離數字
                cv2.rectangle(depth_vis, (x1, y1), (x2, y2), (255, 255, 255), 1)
                if min_dist > 0:
                    cv2.putText(
                        depth_vis,
                        f'{min_dist * 100:.1f}cm',
                        (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 1
                    )
            else:
                depth_vis = np.zeros((h, w, 3), dtype=np.uint8)

            vis_msg = self.bridge.cv2_to_imgmsg(depth_vis, encoding='bgr8')
            vis_msg.header = left_msg.header
            self.pub_visual.publish(vis_msg)

            # ── 6. 視差圖（供 RTAB-Map 等使用）──────────────
            disp_msg = self.bridge.cv2_to_imgmsg(disp_raw, encoding='32FC1')
            disp_msg.header = left_msg.header
            self.pub_disp.publish(disp_msg)

        except Exception as e:
            self.get_logger().error(f'image_callback error: {e}')


# ─────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = StereoDepthNode()
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
