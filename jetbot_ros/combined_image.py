#!/usr/bin/env python3
"""
檢查左右相機 rectification 是否正確
執行後會存 /tmp/stereo_check.png
用水平線確認左右影像是否對齊
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class StereoChecker(Node):
    def __init__(self):
        super().__init__('stereo_checker')
        self.bridge = CvBridge()
        self.left  = None
        self.right = None
        self.saved = False

        self.create_subscription(Image, '/camera_left/image_rect',  self.cb_l, 10)
        self.create_subscription(Image, '/camera_right/image_rect', self.cb_r, 10)
        self.get_logger().info('等待左右影像...')

    def cb_l(self, msg):
        self.left = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.try_save()

    def cb_r(self, msg):
        self.right = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.try_save()

    def try_save(self):
        if self.saved:
            return
        if self.left is None or self.right is None:
            return

        self.saved = True

        # 左右並排
        combined = np.hstack([self.left, self.right])
        w = combined.shape[1]

        # 每隔 40px 畫綠色水平線（兩張影像對齊的話，線應該貫穿場景同一高度）
        for y in range(0, combined.shape[0], 40):
            cv2.line(combined, (0, y), (w, y), (0, 255, 0), 1)

        # 中間畫紅色分隔線
        mid = self.left.shape[1]
        cv2.line(combined, (mid, 0), (mid, combined.shape[0]), (0, 0, 255), 2)

        # 標籤
        cv2.putText(combined, 'LEFT (rect)',  (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(combined, 'RIGHT (rect)', (mid + 10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        path = '/tmp/stereo_check.png'
        cv2.imwrite(path, combined)
        self.get_logger().info(f'✅ 存檔完成: {path}')
        self.get_logger().info('把 /tmp/stereo_check.png 傳到電腦上傳給 Claude 看')
        raise SystemExit


def main():
    rclpy.init()
    node = StereoChecker()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()