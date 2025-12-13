#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
立体相机深度估计节点
功能: 从左右相机图像计算深度并发布深度图
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import message_filters
import yaml
import os


class StereoDepthNode(Node):
    def __init__(self):
        super().__init__('stereo_depth_node')

        # 声明参数
        self.declare_parameter('left_camera_config', '/home/user/jetbot_ros/config/left.yaml')
        self.declare_parameter('right_camera_config', '/home/user/jetbot_ros/config/right.yaml')
        self.declare_parameter('baseline', 0.11)  # 11 公分
        self.declare_parameter('min_disparity', 0)
        self.declare_parameter('num_disparities', 64)
        self.declare_parameter('block_size', 15)

        # 读取参数
        left_config_path = self.get_parameter('left_camera_config').value
        right_config_path = self.get_parameter('right_camera_config').value
        self.baseline = self.get_parameter('baseline').value

        # 读取相机参数
        self.left_camera_info = self.load_camera_config(left_config_path)
        self.right_camera_info = self.load_camera_config(right_config_path)

        if self.left_camera_info is None or self.right_camera_info is None:
            self.get_logger().error('无法加载相机配置文件')
            raise RuntimeError('相机配置加载失败')

        # 焦距参数
        self.focal_length = self.left_camera_info['fx']
        self.cx = self.left_camera_info['cx']
        self.cy = self.left_camera_info['cy']

        self.get_logger().info('='*50)
        self.get_logger().info('深度估计节点参数:')
        self.get_logger().info(f'  Baseline: {self.baseline} m')
        self.get_logger().info(f'  焦距 fx: {self.focal_length:.2f} px')
        self.get_logger().info('='*50)

        # 立体匹配参数
        self.min_disparity = self.get_parameter('min_disparity').value
        self.num_disparities = self.get_parameter('num_disparities').value
        self.block_size = self.get_parameter('block_size').value

        # CV Bridge
        self.bridge = CvBridge()

        # 创建立体匹配器
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=self.min_disparity,
            numDisparities=self.num_disparities,
            blockSize=self.block_size,
            P1=8 * 3 * self.block_size ** 2,
            P2=32 * 3 * self.block_size ** 2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )

        # 订阅左右相机
        left_sub = message_filters.Subscriber(
            self, Image, 'camera_left/image_rect'
        )
        right_sub = message_filters.Subscriber(
            self, Image, 'camera_right/image_rect'
        )

        # 时间同步
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [left_sub, right_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.stereo_callback)

        # 发布深度图 (32位浮点数，单位：米)
        self.depth_pub = self.create_publisher(
            Image,
            'stereo/depth',
            10
        )

        # 发布平均深度值
        self.avg_depth_pub = self.create_publisher(
            Float32,
            'stereo/avg_depth',
            10
        )

        # 发布视差图（用于调试）
        self.disparity_pub = self.create_publisher(
            Image,
            'stereo/disparity',
            10
        )

        self.get_logger().info('深度估计节点已启动')
        self.get_logger().info('订阅: camera_left/image_rect, camera_right/image_rect')
        self.get_logger().info('发布: stereo/depth, stereo/avg_depth, stereo/disparity')

    def load_camera_config(self, config_path):
        """从 YAML 文件加载相机配置"""
        try:
            if not os.path.exists(config_path):
                self.get_logger().error(f'配置文件不存在: {config_path}')
                return None

            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

            camera_info = {}

            if 'camera_matrix' in config:
                K = config['camera_matrix']['data']
                camera_info['fx'] = K[0]
                camera_info['fy'] = K[4]
                camera_info['cx'] = K[2]
                camera_info['cy'] = K[5]
            else:
                self.get_logger().error(f'无法识别的 YAML 格式: {config_path}')
                return None

            if not all(k in camera_info for k in ['fx', 'fy', 'cx', 'cy']):
                self.get_logger().error(f'配置文件缺少必要参数')
                return None

            return camera_info

        except Exception as e:
            self.get_logger().error(f'加载配置文件时出错: {e}')
            return None

    def stereo_callback(self, left_msg, right_msg):
        """处理左右相机图像"""
        try:
            # 转换为 OpenCV 格式
            left_image = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='mono8')
            right_image = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='mono8')

            # 计算视差图
            disparity = self.stereo.compute(left_image, right_image).astype(np.float32) / 16.0

            # 计算深度图: depth = (baseline * focal_length) / disparity
            depth_map = np.zeros_like(disparity, dtype=np.float32)
            valid_mask = disparity > 0
            depth_map[valid_mask] = (self.baseline * self.focal_length) / disparity[valid_mask]

            # 限制深度范围 (0.1m 到 10m)
            depth_map = np.clip(depth_map, 0.1, 10.0)

            # 发布深度图
            self.publish_depth_map(depth_map, left_msg.header)

            # 计算并发布平均深度
            valid_depths = depth_map[valid_mask]
            if len(valid_depths) > 0:
                avg_depth = float(np.mean(valid_depths))
                avg_depth_msg = Float32()
                avg_depth_msg.data = avg_depth
                self.avg_depth_pub.publish(avg_depth_msg)

                self.get_logger().info(
                    f'平均深度: {avg_depth:.2f} m',
                    throttle_duration_sec=1.0
                )

            # 发布视差图（调试用）
            self.publish_disparity(disparity, left_msg.header)

        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge 错误: {e}')
        except Exception as e:
            self.get_logger().error(f'处理图像时出错: {e}')

    def publish_depth_map(self, depth_map, header):
        """发布深度图 (32位浮点数图像)"""
        try:
            depth_msg = self.bridge.cv2_to_imgmsg(depth_map, encoding='32FC1')
            depth_msg.header = header
            depth_msg.header.frame_id = 'camera_left'
            self.depth_pub.publish(depth_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'发布深度图时出错: {e}')

    def publish_disparity(self, disparity, header):
        """发布视差图 (用于可视化)"""
        # 归一化到 0-255
        disparity_normalized = cv2.normalize(
            disparity,
            None,
            alpha=0,
            beta=255,
            norm_type=cv2.NORM_MINMAX,
            dtype=cv2.CV_8U
        )

        try:
            disparity_msg = self.bridge.cv2_to_imgmsg(disparity_normalized, encoding='mono8')
            disparity_msg.header = header
            self.disparity_pub.publish(disparity_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'发布视差图时出错: {e}')


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
