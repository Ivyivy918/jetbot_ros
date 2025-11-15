#!/usr/bin/env python3
"""
JetBot 完整系統啟動檔案
功能: 雙鏡頭 + 鍵盤控制 + RTAB-Map SLAM + RViz2 視覺化 + 機器人模型
使用: ros2 launch jetbot_ros jetbot.launch.py
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 取得套件路徑
    pkg_share = get_package_share_directory('jetbot_ros')
    rviz_config = os.path.join(pkg_share, 'rviz', 'jetbot_mapping.rviz')
    urdf_file = os.path.join(pkg_share, 'urdf', 'jetbot.urdf.xacro')
    
    # 引入立體相機 launch 檔
    stereo_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'stereo_camera.launch.py')
        )
    )
    
    nodes_to_launch = [
        # ========== 引入立體相機系統 ==========
        stereo_camera_launch,
        
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
        ),
        
        # ========== Joint State Publisher ==========
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'source_list': ['left_wheel_joint', 'right_wheel_joint']
            }]
        ),
        
        # ========== Motor Node (含里程計) ==========
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
                ('left/image_rect', '/stereo/left/image_rect_color'),
                ('right/image_rect', '/stereo/right/image_rect_color'),
                ('left/camera_info', '/stereo/left/camera_info'),
                ('right/camera_info', '/stereo/right/camera_info'),
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

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_left',
            arguments=['0.095', '0.08', '0.19', '0', '0', '0', 
                    'base_footprint', 'camera_left_optical_frame'],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_right',
            arguments=['0.095', '-0.08', '0.19', '0', '0', '0', 
                    'base_footprint', 'camera_right_optical_frame'],
            output='screen'
        ),
    ]
    
    # 過濾掉 None 的節點
    return LaunchDescription([n for n in nodes_to_launch if n is not None])