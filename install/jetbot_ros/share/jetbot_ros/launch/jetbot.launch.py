#!/usr/bin/env python3
"""
JetBot 邊緣運算啟動檔案 (Headless 版本)
功能: 雙鏡頭發布 + 深度估計 + 馬達控制 + RTAB-Map 背景 SLAM
注意: Rviz2 與鍵盤控制請在遠端 PC (Ivy) 上執行
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('jetbot_ros')
    urdf_file = os.path.join(pkg_share, 'urdf', 'jetbot.urdf.xacro')
    camera_left_yaml = os.path.join(pkg_share, 'config/left.yaml')
    camera_right_yaml = os.path.join(pkg_share, 'config/right.yaml')
    
    return LaunchDescription([
        # ========== 機器人狀態發布 ==========
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file]),
                'use_sim_time': False
            }]
        ) if os.path.exists(urdf_file) else None,

        # ========== 雙相機擷取節點 (無螢幕需求) ==========
        Node(
            package='jetbot_ros',
            executable='camera_node',
            name='stereo_pointcloud_node',
            output='screen'
            # 移除了強制找 DISPLAY 的環境變數
        ),

        # ========== 深度估計節點 ==========
        Node(
            package='jetbot_ros',
            executable='stereo_depth_node',
            name='stereo_depth_node',
            output='screen',
            parameters=[{
                'left_camera_config': camera_left_yaml,
                'right_camera_config': camera_right_yaml,
                'baseline': 0.11,
                'min_disparity': 0,
                'num_disparities': 64,
                'block_size': 15
            }]
        ),

        # ========== 馬達控制節點 ==========
        Node(
            package='jetbot_ros',
            executable='motors_node',
            name='motors_node',
            output='screen'
        ),
        
        # ========== RTAB-Map SLAM (背景運算) ==========
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'subscribe_stereo': True,
                'subscribe_depth': False,
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'approx_sync': True,
                'queue_size': 30,
                'Rtabmap/DetectionRate': '2.0',
                'RGBD/AngularUpdate': '0.05',
                'RGBD/LinearUpdate': '0.05',
                'RGBD/OptimizeFromGraphEnd': 'false',
                'Optimizer/Strategy': '0',
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false',
                'Stereo/MaxDisparity': '256',
                'Stereo/MinDisparity': '1',
                'Stereo/OpticalFlow': 'true',
                'Vis/FeatureType': '6',
                'Vis/MaxFeatures': '1000',
                'Vis/MinInliers': '10',
                'Vis/InlierDistance': '0.1',
                'Kp/DetectorStrategy': '6',
                'Kp/MaxFeatures': '400',
            }],
            remappings=[
                ('left/image_rect', '/camera_left/image_raw'),
                ('right/image_rect', '/camera_right/image_raw'),
                ('left/camera_info', '/camera_left/camera_info'),
                ('right/camera_info', '/camera_right/camera_info'),
            ],
            arguments=['--delete_db_on_start']
        ),
    ])