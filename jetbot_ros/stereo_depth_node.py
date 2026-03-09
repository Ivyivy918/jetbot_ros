#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters

class StereoDepthNode(Node):
    def __init__(self):
        super().__init__('stereo_depth_node')
        self.bridge = CvBridge()

        # 訂閱左眼與右眼的影像
        self.left_sub = message_filters.Subscriber(self, Image, '/camera_left/image_raw')
        self.right_sub = message_filters.Subscriber(self, Image, '/camera_right/image_raw')

        # 時間同步器：確保左右眼是在同一個瞬間拍下的畫面 (極度重要！)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub], 10, 0.1)
        self.ts.registerCallback(self.image_callback)

        # 準備發佈計算好的深度圖
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)

        # OpenCV 的立體視覺匹配演算法 (SGBM)
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=64,
            blockSize=15,
            P1=8 * 3 * 15 ** 2,
            P2=32 * 3 * 15 ** 2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32
        )
        self.get_logger().info('✅ Stereo Depth Node initialized. Ready to calculate depth!')

    def image_callback(self, left_msg, right_msg):
        try:
            # 轉換 ROS 影像為 OpenCV 陣列
            left_img = self.bridge.imgmsg_to_cv2(left_msg, "mono8")
            right_img = self.bridge.imgmsg_to_cv2(right_msg, "mono8")

            # 計算視差圖
            disparity = self.stereo.compute(left_img, right_img).astype(np.float32) / 16.0

            # 轉換成可視化的 8-bit 灰階圖發佈出去
            disp_visual = cv2.normalize(disparity, disparity, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            
            depth_msg = self.bridge.cv2_to_imgmsg(disp_visual, "mono8")
            depth_msg.header = left_msg.header
            self.depth_pub.publish(depth_msg)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = StereoDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
