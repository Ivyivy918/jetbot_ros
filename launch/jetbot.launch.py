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
    
    # 相機校正檔案路徑
    camera_left_yaml = os.path.join(pkg_share, 'config/camera_calibration/left/camera_left.yaml')
    camera_right_yaml = os.path.join(pkg_share, 'config/camera_calibration/right/camera_right.yaml')
    
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
        
        # ========== TF Transforms ==========
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
            output='screen'
        ),
        
        # ========== Camera Node ==========
        Node(
            package='jetbot_ros',
            executable='camera_node',
            name='camera_node',
            output='screen'
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
                'Rtabmap/DetectionRate': '1.0',
                'RGBD/AngularUpdate': '0.01',
                'RGBD/LinearUpdate': '0.01',
                'RGBD/OptimizeFromGraphEnd': 'false',
                'Optimizer/Strategy': '0',
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false',
            }],
            remappings=[
                ('left/image_rect', '/camera_left/image_raw'),
                ('right/image_rect', '/camera_right/image_raw'),
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
