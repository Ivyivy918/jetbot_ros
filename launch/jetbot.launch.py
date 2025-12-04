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
    camera_left_yaml = os.path.join(pkg_share, 'config/left.yaml')
    camera_right_yaml = os.path.join(pkg_share, 'config/right.yaml')
    
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
        

        # ========== Camera Node ==========
        Node(
            package='jetbot_ros',
            executable='camera_node',
            name='camera_node',
            output='screen',
            additional_env={'DISPLAY': ':1'}
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
                
                # 降低處理頻率
                'Rtabmap/DetectionRate': '2.0',
                
                # 優化參數
                'RGBD/AngularUpdate': '0.05',
                'RGBD/LinearUpdate': '0.05',
                'RGBD/OptimizeFromGraphEnd': 'false',
                'Optimizer/Strategy': '0',
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false',
                
                # ===== 關鍵：立體視覺參數 =====
                'Stereo/MaxDisparity': '256',  # 增加視差範圍
                'Stereo/MinDisparity': '1',
                'Stereo/OpticalFlow': 'true',
                
                # ===== 關鍵：特徵提取參數 =====
                'Vis/FeatureType': '6',  # GFTT
                'Vis/MaxFeatures': '1000',
                'Vis/MinInliers': '10',  # 降低要求
                'Vis/InlierDistance': '0.1',
                
                # ===== 回環檢測 =====
                'Kp/DetectorStrategy': '6',  # GFTT
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

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{
                'source_list': ['left_wheel_joint', 'right_wheel_joint']
            }]
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
