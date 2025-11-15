#!/usr/bin/env python3
"""
立體相機啟動檔案 - Jetson CSI 相機版本
"""
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_share = get_package_share_directory('jetbot_ros')
    
    # 校正檔案路徑
    camera_left_yaml = os.path.join(pkg_share, 'config', 'camera_calibration', 'left', 'camera_left.yaml')
    camera_right_yaml = os.path.join(pkg_share, 'config', 'camera_calibration', 'right', 'camera_right.yaml')
    
    return LaunchDescription([
        # ========== 左相機 ==========
        Node(
            package='jetbot_ros',
            executable='csi_camera_node',
            name='camera_left',
            namespace='stereo/left',
            output='screen',
            parameters=[{
                'sensor_id': 1,
                'width': 640,
                'height': 480,
                'fps': 30,
                'camera_frame_id': 'camera_left_optical_frame',
                'camera_info_url': f'file://{camera_left_yaml}' if os.path.exists(camera_left_yaml) else '',
            }]
        ),
        
        # ========== 右相機 ==========
        Node(
            package='jetbot_ros',
            executable='csi_camera_node',
            name='camera_right',
            namespace='stereo/right',
            output='screen',
            parameters=[{
                'sensor_id': 0,
                'width': 640,
                'height': 480,
                'fps': 30,
                'camera_frame_id': 'camera_right_optical_frame',
                'camera_info_url': f'file://{camera_right_yaml}' if os.path.exists(camera_right_yaml) else '',
            }]
        ),
        
        # ========== 左影像矯正 ==========
        Node(
            package='image_proc',
            executable='image_proc',
            name='image_proc_left',
            namespace='stereo/left',
            output='screen',
            remappings=[
                ('image', 'image_raw'),
                ('camera_info', 'camera_info'),
            ]
        ),
        
        # ========== 右影像矯正 ==========
        Node(
            package='image_proc',
            executable='image_proc',
            name='image_proc_right',
            namespace='stereo/right',
            output='screen',
            remappings=[
                ('image', 'image_raw'),
                ('camera_info', 'camera_info'),
            ]
        ),
        
        # ========== 立體影像處理容器 ==========
        ComposableNodeContainer(
            name='stereo_container',
            namespace='stereo',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # 視差計算
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::DisparityNode',
                    name='disparity_node',
                    remappings=[
                        ('left/image_rect', '/stereo/left/image_rect'),
                        ('left/camera_info', '/stereo/left/camera_info'),
                        ('right/image_rect', '/stereo/right/image_rect'),
                        ('right/camera_info', '/stereo/right/camera_info'),
                    ],
                    parameters=[{
                        'approximate_sync': True,
                    }]
                ),
                # 點雲生成
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::PointCloudNode',
                    name='point_cloud_node',
                    remappings=[
                        ('left/image_rect_color', '/stereo/left/image_rect_color'),
                        ('left/camera_info', '/stereo/left/camera_info'),
                        ('right/camera_info', '/stereo/right/camera_info'),
                        ('disparity', '/stereo/disparity'),
                    ],
                    parameters=[{
                        'approximate_sync': True,
                    }]
                ),
            ],
            output='screen',
        ),
    ])