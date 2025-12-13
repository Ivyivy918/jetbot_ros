#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
双相机图像合成节点
功能: 使用视差信息生成中间视角的高质量合成图像
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import message_filters
import os
from datetime import datetime


class CombinedImageNode(Node):
    def __init__(self):
        super().__init__('combined_image_node')
        
        # 声明参数
        self.declare_parameter('show_window', True)
        self.declare_parameter('auto_save', False)
        self.declare_parameter('save_path', '~/jetbot_ros/captured_images')
        self.declare_parameter('save_interval', 30)
        self.declare_parameter('use_disparity', True)  # 使用视差信息
        
        self.show_window = self.get_parameter('show_window').value
        self.auto_save = self.get_parameter('auto_save').value
        save_path = self.get_parameter('save_path').value
        self.save_interval = self.get_parameter('save_interval').value
        self.use_disparity = self.get_parameter('use_disparity').value
        
        # 展开路径
        self.save_path = os.path.expanduser(save_path)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 图像计数器
        self.frame_count = 0
        self.saved_count = 0
        
        # 创建立体匹配器（用于计算视差）
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=64,
            blockSize=11,
            P1=8 * 3 * 11 ** 2,
            P2=32 * 3 * 11 ** 2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )
        
        # 创建保存目录
        os.makedirs(self.save_path, exist_ok=True)
        
        # 订阅左右相机图像
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
        self.ts.registerCallback(self.image_callback)
        
        # 发布合成图像
        self.combined_pub = self.create_publisher(
            Image,
            'stereo/combined_image',
            10
        )
        
        self.get_logger().info('='*60)
        self.get_logger().info('双相机中间视角合成节点已启动 (改进版)')
        self.get_logger().info('使用基于视差的视图合成算法')
        self.get_logger().info(f'相机间距: 15 公分')
        self.get_logger().info(f'保存路径: {self.save_path}')
        if self.show_window:
            self.get_logger().info('按键控制:')
            self.get_logger().info('  [q] 退出  [s] 保存  [d] 切换视差模式')
        self.get_logger().info('='*60)
    
    def synthesize_center_view(self, left, right):
        """使用视差信息合成中间视角"""
        # 转换为灰度图计算视差
        left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        
        # 计算视差图
        disparity = self.stereo.compute(left_gray, right_gray).astype(np.float32) / 16.0
        
        # 归一化视差到 0-1
        disp_norm = cv2.normalize(disparity, None, 0, 1, cv2.NORM_MINMAX)
        
        h, w = left.shape[:2]
        
        # 创建输出图像
        center_view = np.zeros_like(left)
        
        # 对每个像素，根据视差进行视图插值
        for y in range(h):
            for x in range(w):
                d = disparity[y, x]
                
                if d > 0:  # 有效视差
                    # 计算中间视角的像素位置
                    # 中间视角在左右相机之间，所以偏移量是视差的一半
                    offset = d / 2.0
                    
                    # 从左图向右偏移
                    x_left = int(x + offset)
                    # 从右图向左偏移  
                    x_right = int(x - offset)
                    
                    # 收集有效像素
                    pixels = []
                    
                    if 0 <= x_left < w:
                        pixels.append(left[y, x_left])
                    
                    if 0 <= x_right < w:
                        pixels.append(right[y, x_right])
                    
                    if len(pixels) > 0:
                        # 平均融合
                        center_view[y, x] = np.mean(pixels, axis=0).astype(np.uint8)
                    else:
                        # 无有效像素，使用左图
                        center_view[y, x] = left[y, x]
                else:
                    # 无有效视差，使用简单平均
                    center_view[y, x] = ((left[y, x].astype(np.float32) + 
                                         right[y, x].astype(np.float32)) / 2).astype(np.uint8)
        
        return center_view
    
    def synthesize_center_view_fast(self, left, right):
        """快速版本：使用加权平均和边缘保持"""
        # 转换为灰度图计算视差
        left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        
        # 计算视差图
        disparity = self.stereo.compute(left_gray, right_gray).astype(np.float32) / 16.0
        
        # 创建权重图（基于视差的可靠性）
        valid_mask = disparity > 0
        
        # 使用导向滤波平滑视差图
        disparity_filtered = cv2.ximgproc.guidedFilter(left_gray, disparity, 15, 0.01)
        
        h, w = left.shape[:2]
        
        # 基于视差创建动态权重
        # 视差越大，说明物体越近，我们需要更仔细的处理
        weight = np.clip(disparity_filtered / 32.0, 0, 1)
        weight = np.stack([weight] * 3, axis=2)  # 扩展到RGB
        
        # 使用平滑的权重混合
        center_view = (left.astype(np.float32) * (1 - weight * 0.3) + 
                      right.astype(np.float32) * (1 - (1 - weight) * 0.3))
        center_view = center_view / 2.0
        center_view = np.clip(center_view, 0, 255).astype(np.uint8)
        
        # 使用双边滤波减少伪影
        center_view = cv2.bilateralFilter(center_view, 5, 50, 50)
        
        return center_view
    
    def synthesize_simple_advanced(self, left, right):
        """改进的简单混合：使用边缘保持"""
        # 检测边缘
        left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(left_gray, 50, 150)
        edges = cv2.dilate(edges, np.ones((3, 3), np.uint8))
        
        # 在边缘区域使用左图，其他区域混合
        edge_mask = (edges > 0).astype(np.float32)
        edge_mask = cv2.GaussianBlur(edge_mask, (11, 11), 0)
        edge_mask = np.stack([edge_mask] * 3, axis=2)
        
        # 混合
        blended = (left.astype(np.float32) * edge_mask + 
                  (left.astype(np.float32) * 0.5 + right.astype(np.float32) * 0.5) * (1 - edge_mask))
        
        return blended.astype(np.uint8)
    
    def save_image(self, image, prefix='manual'):
        """保存图像"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        filename = f'{prefix}_{timestamp}_{self.saved_count:04d}.jpg'
        filepath = os.path.join(self.save_path, filename)
        
        cv2.imwrite(filepath, image)
        self.saved_count += 1
        
        self.get_logger().info(f'✓ 已保存: {filename}')
        return filepath
    
    def image_callback(self, left_msg, right_msg):
        """处理左右相机图像并合成"""
        try:
            # 转换 ROS 图像为 OpenCV 格式
            left_image = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='bgr8')
            right_image = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='bgr8')
            
            # 确保两张图像尺寸一致
            h1, w1 = left_image.shape[:2]
            h2, w2 = right_image.shape[:2]
            
            if h1 != h2 or w1 != w2:
                right_image = cv2.resize(right_image, (w1, h1))
            
            # 合成中间视角（使用快速版本）
            if self.use_disparity:
                combined = self.synthesize_center_view_fast(left_image, right_image)
            else:
                combined = self.synthesize_simple_advanced(left_image, right_image)
            
            # 添加信息文字
            h, w = combined.shape[:2]
            font = cv2.FONT_HERSHEY_SIMPLEX
            
            mode_text = 'Disparity-based' if self.use_disparity else 'Edge-preserving'
            cv2.putText(combined, f'Center View ({mode_text})', (10, 40), 
                       font, 1.0, (0, 255, 0), 2)
            
            # 帧计数
            self.frame_count += 1
            info_text = f'Frame: {self.frame_count} | Saved: {self.saved_count}'
            cv2.putText(combined, info_text, (10, h - 15), 
                       font, 0.6, (255, 255, 255), 2)
            
            # 自动保存
            if self.auto_save and self.frame_count % self.save_interval == 0:
                self.save_image(combined, prefix='auto')
            
            # 显示图像窗口
            if self.show_window:
                cv2.imshow('Stereo Camera - Center View', combined)
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    self.get_logger().info('用户按下 q 键，关闭节点')
                    cv2.destroyAllWindows()
                    rclpy.shutdown()
                    
                elif key == ord('s'):
                    self.save_image(combined, prefix='manual')
                    
                elif key == ord('d'):
                    self.use_disparity = not self.use_disparity
                    mode = 'Disparity-based' if self.use_disparity else 'Edge-preserving'
                    self.get_logger().info(f'切换模式: {mode}')
            
            # 发布
            combined_msg = self.bridge.cv2_to_imgmsg(combined, encoding='bgr8')
            combined_msg.header = left_msg.header
            self.combined_pub.publish(combined_msg)
            
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge 错误: {e}')
        except Exception as e:
            self.get_logger().error(f'处理图像时出错: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CombinedImageNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
