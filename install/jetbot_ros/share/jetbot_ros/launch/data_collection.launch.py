#!/usr/bin/env python3
"""
JetBot 完整系統啟動檔案
功能: 雙鏡頭 + 鍵盤控制 + RTAB-Map SLAM + RViz2 視覺化 + 機器人模型
使用: ros2 launch jetbot_ros jetbot.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 取得套件路徑
    pkg_share = get_package_share_directory('jetbot_ros')
    rviz_config = os.path.join(pkg_share, 'rviz', 'jetbot_mapping.rviz')
    urdf_file = os.path.join(pkg_share, 'urdf', 'jetbot.urdf.xacro')
    
    return LaunchDescription([
        # ========== Robot State Publisher ==========
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
        
        # ========== Stereo Camera Node ==========
        Node(
            package='jetbot_ros',
            executable='camera_node',
            name='stereo_camera',
            output='screen',
            additional_env={'DISPLAY': ':0'}
        ),
        
        # ========== Motor Node ==========
        Node(
            package='jetbot_ros',
            executable='motors_node',
            name='motors_node',
            output='screen'
        ),
        
        # ========== Teleop Keyboard ==========
        Node(
            package='jetbot_ros',
            executable='teleop_keyboard',
            name='teleop_keyboard',
            output='screen',
            prefix='xterm -e',
            emulate_tty=True
        ),
        
        # ========== RTAB-Map SLAM ==========
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
                'Rtabmap/DetectionRate': 2.0,
                'RGBD/AngularUpdate': 0.05,
                'RGBD/LinearUpdate': 0.05,
                'RGBD/OptimizeFromGraphEnd': False,
                'Optimizer/Strategy': 0,
                'Mem/IncrementalMemory': True,
                'Mem/InitWMWithAllNodes': False,
                'Stereo/MaxDisparity': 256,
                'Stereo/MinDisparity': 1,
                'Stereo/OpticalFlow': True,
                'Vis/FeatureType': 6,   # GFTT
                'Vis/MaxFeatures': 1000,
                'Vis/MinInliers': 10,
                'Vis/InlierDistance': 0.1,
                'Kp/DetectorStrategy': 6,  # GFTT
                'Kp/MaxFeatures': 400,
            }],
            remappings=[
                ('left/image_rect', '/camera_left/image_rect'),
                ('right/image_rect', '/camera_right/image_rect'),
                ('left/camera_info', '/camera_left/camera_info'),
                ('right/camera_info', '/camera_right/camera_info'),
            ],
            arguments=['--delete_db_on_start']
        ),
        
        # ========== RViz2 ==========
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config] if os.path.exists(rviz_config) else []
        ),
    ])
