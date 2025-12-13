#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
立体相机点云生成节点
功能: 从左右相机图像生成点云，从 YAML 配置文件读取相机参数
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import struct
import message_filters
import yaml
import os


class StereoPointCloudNode(Node):
    def __init__(self):
        super().__init__('stereo_pointcloud_node')
        
        # 声明参数 - 配置文件路径
        self.declare_parameter('left_camera_config', '/home/user/jetbot_ros/config/left.yaml')
        self.declare_parameter('right_camera_config', '/home/user/jetbot_ros/config/right.yaml')
        self.declare_parameter('baseline', 0.11)  # 11 公分 = 0.11 米
        self.declare_parameter('min_disparity', 0)
        self.declare_parameter('num_disparities', 64)  # 必须是 16 的倍数
        self.declare_parameter('block_size', 15)  # 必须是奇数
        
        # 读取配置文件路径
        left_config_path = self.get_parameter('left_camera_config').value
        right_config_path = self.get_parameter('right_camera_config').value
        self.baseline = self.get_parameter('baseline').value
        
        # 读取相机参数
        self.left_camera_info = self.load_camera_config(left_config_path)
        self.right_camera_info = self.load_camera_config(right_config_path)
        
        if self.left_camera_info is None or self.right_camera_info is None:
            self.get_logger().error('无法加载相机配置文件，节点退出')
            raise RuntimeError('相机配置加载失败')
        
        # 从左相机获取内参 (用于点云计算)
        self.focal_length = self.left_camera_info['fx']  # 焦距
        self.cx = self.left_camera_info['cx']  # 主点 x
        self.cy = self.left_camera_info['cy']  # 主点 y
        
        self.get_logger().info('='*50)
        self.get_logger().info('立体相机参数:')
        self.get_logger().info(f'  左相机配置: {left_config_path}')
        self.get_logger().info(f'  右相机配置: {right_config_path}')
        self.get_logger().info(f'  Baseline: {self.baseline} m')
        self.get_logger().info(f'  左相机 fx: {self.left_camera_info["fx"]:.2f} px')
        self.get_logger().info(f'  左相机 fy: {self.left_camera_info["fy"]:.2f} px')
        self.get_logger().info(f'  左相机 cx: {self.left_camera_info["cx"]:.2f} px')
        self.get_logger().info(f'  左相机 cy: {self.left_camera_info["cy"]:.2f} px')
        self.get_logger().info('='*50)
        
        # 立体匹配参数
        self.min_disparity = self.get_parameter('min_disparity').value
        self.num_disparities = self.get_parameter('num_disparities').value
        self.block_size = self.get_parameter('block_size').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 创建立体匹配器 (使用 Semi-Global Block Matching)
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
        
        # 订阅左右相机图像 (使用 message_filters 进行时间同步)
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
            slop=0.1  # 允许 0.1 秒的时间差
        )
        self.ts.registerCallback(self.stereo_callback)
        
        # 发布点云
        self.pointcloud_pub = self.create_publisher(
            PointCloud2, 
            'stereo/pointcloud', 
            10
        )
        
        # 发布视差图 (用于调试)
        self.disparity_pub = self.create_publisher(
            Image,
            'stereo/disparity',
            10
        )
        
        self.get_logger().info('立体点云节点已启动')
        self.get_logger().info('订阅主题:')
        self.get_logger().info('  - camera_left/image_rect')
        self.get_logger().info('  - camera_right/image_rect')
        self.get_logger().info('发布主题:')
        self.get_logger().info('  - stereo/pointcloud')
        self.get_logger().info('  - stereo/disparity')
    
    def load_camera_config(self, config_path):
        """从 YAML 文件加载相机配置"""
        try:
            if not os.path.exists(config_path):
                self.get_logger().error(f'配置文件不存在: {config_path}')
                return None
            
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            # 支持两种常见的 YAML 格式
            camera_info = {}
            
            # 格式 1: 直接包含 camera_matrix
            if 'camera_matrix' in config:
                K = config['camera_matrix']['data']
                # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
                camera_info['fx'] = K[0]
                camera_info['fy'] = K[4]
                camera_info['cx'] = K[2]
                camera_info['cy'] = K[5]
            
            # 格式 2: 包含在某个相机名称下
            elif 'camera_name' in config:
                # 寻找 camera_matrix
                for key in config:
                    if isinstance(config[key], dict) and 'data' in config[key]:
                        if key == 'camera_matrix':
                            K = config[key]['data']
                            camera_info['fx'] = K[0]
                            camera_info['fy'] = K[4]
                            camera_info['cx'] = K[2]
                            camera_info['cy'] = K[5]
                            break
            
            # 格式 3: 直接包含 fx, fy, cx, cy
            elif 'fx' in config:
                camera_info['fx'] = config['fx']
                camera_info['fy'] = config['fy']
                camera_info['cx'] = config['cx']
                camera_info['cy'] = config['cy']
            
            # 格式 4: ROS2 标准格式
            elif 'image_width' in config and 'camera_matrix' in config:
                K = config['camera_matrix']['data']
                camera_info['fx'] = K[0]
                camera_info['fy'] = K[4]
                camera_info['cx'] = K[2]
                camera_info['cy'] = K[5]
                camera_info['width'] = config['image_width']
                camera_info['height'] = config['image_height']
            
            else:
                self.get_logger().error(f'无法识别的 YAML 格式: {config_path}')
                self.get_logger().error(f'配置内容: {config}')
                return None
            
            # 验证参数
            if not all(k in camera_info for k in ['fx', 'fy', 'cx', 'cy']):
                self.get_logger().error(f'配置文件缺少必要参数: {config_path}')
                return None
            
            self.get_logger().info(f'✓ 成功加载配置: {config_path}')
            
            return camera_info
            
        except Exception as e:
            self.get_logger().error(f'加载配置文件时出错 ({config_path}): {e}')
            return None
    
    def stereo_callback(self, left_msg, right_msg):
        """处理左右相机图像"""
        try:
            # 转换 ROS 图像为 OpenCV 格式
            left_image = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='mono8')
            right_image = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='mono8')
            
            # 计算视差图
            disparity = self.stereo.compute(left_image, right_image).astype(np.float32) / 16.0
            
            # 发布视差图 (用于调试)
            self.publish_disparity(disparity, left_msg.header)
            
            # 生成点云
            points = self.disparity_to_pointcloud(
                disparity, 
                left_image,
                self.baseline,
                self.focal_length,
                self.cx,
                self.cy
            )
            
            # 发布点云
            if points is not None and len(points) > 0:
                pointcloud_msg = self.create_pointcloud2(points, left_msg.header)
                self.pointcloud_pub.publish(pointcloud_msg)
                self.get_logger().info(
                    f'发布点云: {len(points)} 个点',
                    throttle_duration_sec=1.0
                )
            
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge 错误: {e}')
        except Exception as e:
            self.get_logger().error(f'处理图像时出错: {e}')
    
    def disparity_to_pointcloud(self, disparity, image, baseline, focal_length, cx, cy):
        """将视差图转换为点云"""
        h, w = disparity.shape
        
        # 创建坐标网格
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        
        # 过滤无效视差值
        valid_mask = (disparity > 0) & (disparity < self.num_disparities)
        
        # 计算深度 Z = (baseline * focal_length) / disparity
        depth = np.zeros_like(disparity)
        depth[valid_mask] = (baseline * focal_length) / disparity[valid_mask]
        
        # 计算 3D 坐标
        # X = (u - cx) * Z / focal_length
        # Y = (v - cy) * Z / focal_length
        x = (u - cx) * depth / focal_length
        y = (v - cy) * depth / focal_length
        z = depth
        
        # 提取有效点
        valid_points = valid_mask & (depth < 10.0)  # 限制最大深度为 10 米
        
        x_valid = x[valid_points]
        y_valid = y[valid_points]
        z_valid = z[valid_points]
        
        # 获取强度值 (灰度值)
        intensity = image[valid_points].astype(np.float32) / 255.0
        
        # 组合为点云 (X, Y, Z, Intensity)
        points = np.column_stack((x_valid, y_valid, z_valid, intensity))
        
        return points
    
    def create_pointcloud2(self, points, header):
        """创建 PointCloud2 消息"""
        # 定义点云字段 (X, Y, Z, Intensity)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        # 创建点云消息头
        cloud_header = Header()
        cloud_header.stamp = header.stamp
        cloud_header.frame_id = 'camera_left'  # 使用左相机坐标系
        
        # 将点云数据打包为二进制
        cloud_data = []
        for point in points:
            cloud_data.append(struct.pack('ffff', point[0], point[1], point[2], point[3]))
        
        # 创建 PointCloud2 消息
        pointcloud = PointCloud2()
        pointcloud.header = cloud_header
        pointcloud.height = 1
        pointcloud.width = len(points)
        pointcloud.fields = fields
        pointcloud.is_bigendian = False
        pointcloud.point_step = 16  # 4 个 float32 = 16 字节
        pointcloud.row_step = pointcloud.point_step * pointcloud.width
        pointcloud.data = b''.join(cloud_data)
        pointcloud.is_dense = True
        
        return pointcloud
    
    def publish_disparity(self, disparity, header):
        """发布视差图 (用于可视化)"""
        # 归一化视差图到 0-255
        disparity_normalized = cv2.normalize(
            disparity, 
            None, 
            alpha=0, 
            beta=255, 
            norm_type=cv2.NORM_MINMAX, 
            dtype=cv2.CV_8U
        )
        
        # 转换为 ROS 图像消息
        try:
            disparity_msg = self.bridge.cv2_to_imgmsg(disparity_normalized, encoding='mono8')
            disparity_msg.header = header
            self.disparity_pub.publish(disparity_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'发布视差图时出错: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = StereoPointCloudNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
