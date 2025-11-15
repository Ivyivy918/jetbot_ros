#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_share = get_package_share_directory('jetbot_ros')
    
    # 使用立體校正檔案
    camera_left_yaml = os.path.join(pkg_share, 'config', 'camera_calibration', 'stereo', 'left.yaml')
    camera_right_yaml = os.path.join(pkg_share, 'config', 'camera_calibration', 'stereo', 'right.yaml')
    
    return LaunchDescription([
        # ========== 左相機 ==========
        Node(
            package='jetbot_ros',
            executable='csi_camera_node',
            name='camera_left',
            output='screen',
            parameters=[{
                'sensor_id': 1,
                'width': 1280,
                'height': 720,
                'fps': 30,
                'output_width': 640,
                'output_height': 480,
                'camera_frame_id': 'camera_left_optical_frame',
                'camera_info_url': f'file://{camera_left_yaml}' if os.path.exists(camera_left_yaml) else '',
            }],
            remappings=[
                ('image_raw', '/stereo/left/image_raw'),
                ('camera_info', '/stereo/left/camera_info'),
            ]
        ),
        
        # ========== 右相機 ==========
        Node(
            package='jetbot_ros',
            executable='csi_camera_node',
            name='camera_right',
            output='screen',
            parameters=[{
                'sensor_id': 0,
                'width': 1280,
                'height': 720,
                'fps': 30,
                'output_width': 640,
                'output_height': 480,
                'camera_frame_id': 'camera_right_optical_frame',
                'camera_info_url': f'file://{camera_right_yaml}' if os.path.exists(camera_right_yaml) else '',
            }],
            remappings=[
                ('image_raw', '/stereo/right/image_raw'),
                ('camera_info', '/stereo/right/camera_info'),
            ]
        ),
        
        # ========== 左影像矯正（只用 rectify_color） ==========
        Node(
            package='image_proc',
            executable='rectify_node',  # ← 改這裡！
            name='rectify_left',
            output='screen',
            remappings=[
                ('image', '/stereo/left/image_raw'),
                ('camera_info', '/stereo/left/camera_info'),
                ('image_rect_color', '/stereo/left/image_rect_color'),
            ]
        ),
        
        # ========== 右影像矯正（只用 rectify_color） ==========
        Node(
            package='image_proc',
            executable='rectify_node',  # ← 改這裡！
            name='rectify_right',
            output='screen',
            remappings=[
                ('image', '/stereo/right/image_raw'),
                ('camera_info', '/stereo/right/camera_info'),
                ('image_rect_color', '/stereo/right/image_rect_color'),
            ]
        ),
        
        # ========== 左影像矯正（灰階版本，給視差用） ==========
        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_mono_left',
            output='screen',
            remappings=[
                ('image', '/stereo/left/image_raw'),
                ('camera_info', '/stereo/left/camera_info'),
                ('image_rect', '/stereo/left/image_rect'),
            ]
        ),
        
        # ========== 右影像矯正（灰階版本，給視差用） ==========
        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_mono_right',
            output='screen',
            remappings=[
                ('image', '/stereo/right/image_raw'),
                ('camera_info', '/stereo/right/camera_info'),
                ('image_rect', '/stereo/right/image_rect'),
            ]
        ),
        
        # ========== 視差計算 ==========
        Node(
            package='stereo_image_proc',
            executable='disparity_node',
            name='disparity_node',
            output='screen',
            parameters=[{
                'approximate_sync': True,
            }],
            remappings=[
                ('left/image_rect', '/stereo/left/image_rect'),
                ('left/camera_info', '/stereo/left/camera_info'),
                ('right/image_rect', '/stereo/right/image_rect'),
                ('right/camera_info', '/stereo/right/camera_info'),
                ('disparity', '/stereo/disparity'),
            ]
        ),
        
        # ========== 點雲生成 ==========
        Node(
            package='stereo_image_proc',
            executable='point_cloud_node',
            name='point_cloud_node',
            output='screen',
            parameters=[{
                'approximate_sync': True,
            }],
            remappings=[
                ('left/image_rect_color', '/stereo/left/image_rect_color'),
                ('left/camera_info', '/stereo/left/camera_info'),
                ('right/camera_info', '/stereo/right/camera_info'),
                ('disparity', '/stereo/disparity'),
                ('points2', '/stereo/points2'),
            ]
        ),
    ])