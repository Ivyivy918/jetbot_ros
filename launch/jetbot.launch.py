#!/usr/bin/env python3
"""
JetBot 完整系統啟動檔案 - 一體化版本
所有節點直接定義，不引入其他 launch 檔
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_share = get_package_share_directory('jetbot_ros')
    rviz_config = os.path.join(pkg_share, 'rviz', 'jetbot_mapping.rviz')
    urdf_file = os.path.join(pkg_share, 'urdf', 'jetbot.urdf.xacro')
    
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
        
        # ========== 左影像矯正 ==========
        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_left',
            output='screen',
            remappings=[
                ('image', '/stereo/left/image_raw'),
                ('camera_info', '/stereo/left/camera_info'),
                ('image_rect', '/stereo/left/image_rect'),
            ]
        ),
        
        # ========== 右影像矯正 ==========
        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_right',
            output='screen',
            remappings=[
                ('image', '/stereo/right/image_raw'),
                ('camera_info', '/stereo/right/camera_info'),
                ('image_rect', '/stereo/right/image_rect'),
            ]
        ),
        
        # ========== 左彩色矯正（給 RTAB-Map） ==========
        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_color_left',
            output='screen',
            remappings=[
                ('image', '/stereo/left/image_raw'),
                ('camera_info', '/stereo/left/camera_info'),
                ('image_rect', '/stereo/left/image_rect_color'),
            ]
        ),
        
        # ========== 右彩色矯正（給 RTAB-Map） ==========
        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_color_right',
            output='screen',
            remappings=[
                ('image', '/stereo/right/image_raw'),
                ('camera_info', '/stereo/right/camera_info'),
                ('image_rect', '/stereo/right/image_rect_color'),
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
        ),
        
        # ========== 馬達控制節點 ==========
        Node(
            package='jetbot_ros',
            executable='motors_node',
            name='motors_node',
            output='screen'
        ),
        
        # ========== 鍵盤遙控 ==========
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
                'frame_id': 'body_link',
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'approx_sync': True,
                'queue_size': 30,
                'Rtabmap/DetectionRate': '1.0',
                'RGBD/AngularUpdate': '0.01',
                'RGBD/LinearUpdate': '0.01',
                'RGBD/OptimizeFromGraphEnd': 'false',
                'Optimizer/Strategy': '0',
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false',
            }],
            remappings=[
                ('left/image_rect_color', '/stereo/left/image_rect_color'),
                ('right/image_rect_color', '/stereo/right/image_rect_color'),
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
    ])