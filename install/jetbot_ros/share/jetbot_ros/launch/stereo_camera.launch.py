#!/usr/bin/env python3
"""
立體相機啟動檔案 - Jetson CSI 相機版本 (修正版)
"""
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_share = get_package_share_directory('jetbot_ros')
    
    # 校正檔案路徑
    camera_left_yaml = os.path.join(pkg_share, 'config', 'camera_calibration', 'left', 'camera_left.yaml')
    camera_right_yaml = os.path.join(pkg_share, 'config', 'camera_calibration', 'right', 'camera_right.yaml')
    
    # GStreamer pipeline - 簡化版本
    # 注意:不要用單引號包住 video/x-raw
    gst_pipeline_left = (
        "nvarguscamerasrc sensor-id=1 ! "
        "video/x-raw(memory:NVMM), width=640, height=480, framerate=30/1, format=NV12 ! "
        "nvvidconv flip-method=0 ! "
        "video/x-raw, width=640, height=480, format=BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=BGR"
    )
    
    gst_pipeline_right = (
        "nvarguscamerasrc sensor-id=0 ! "
        "video/x-raw(memory:NVMM), width=640, height=480, framerate=30/1, format=NV12 ! "
        "nvvidconv flip-method=0 ! "
        "video/x-raw, width=640, height=480, format=BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=BGR"
    )
    
    return LaunchDescription([
        # ========== 左相機 ==========
        Node(
            package='gscam',
            executable='gscam_node',
            name='camera_left',
            namespace='stereo/left',
            output='screen',
            parameters=[{
                'gscam_config': gst_pipeline_left,
                'camera_name': 'camera',  # 改成 'camera' 以符合 yaml
                'camera_frame_id': 'camera_left_optical_frame',
                'camera_info_url': f'file://{camera_left_yaml}' if os.path.exists(camera_left_yaml) else '',
            }],
            remappings=[
                ('camera/image_raw', 'image_raw'),
                ('camera/camera_info', 'camera_info'),
            ]
        ),
        
        # ========== 右相機 ==========
        Node(
            package='gscam',
            executable='gscam_node',
            name='camera_right',
            namespace='stereo/right',
            output='screen',
            parameters=[{
                'gscam_config': gst_pipeline_right,
                'camera_name': 'camera',
                'camera_frame_id': 'camera_right_optical_frame',
                'camera_info_url': f'file://{camera_right_yaml}' if os.path.exists(camera_right_yaml) else '',
            }],
            remappings=[
                ('camera/image_raw', 'image_raw'),
                ('camera/camera_info', 'camera_info'),
            ]
        ),
        
        # ========== 視差節點 ==========
        Node(
            package='stereo_image_proc',
            executable='disparity_node',
            name='disparity_node',
            namespace='stereo',
            output='screen',
            parameters=[{
                'approximate_sync': True,
                'queue_size': 10,
            }],
            remappings=[
                ('left/image_rect', 'left/image_rect'),
                ('left/camera_info', 'left/camera_info'),
                ('right/image_rect', 'right/image_rect'),
                ('right/camera_info', 'right/camera_info'),
            ]
        ),
        
        # ========== 點雲節點 ==========
        Node(
            package='stereo_image_proc',
            executable='point_cloud_node',
            name='point_cloud_node',
            namespace='stereo',
            output='screen',
            parameters=[{
                'approximate_sync': True,
                'queue_size': 10,
            }],
            remappings=[
                ('left/image_rect_color', 'left/image_rect_color'),
                ('left/camera_info', 'left/camera_info'),
                ('right/camera_info', 'right/camera_info'),
                ('disparity', 'disparity'),
            ]
        ),
    ])