#!/usr/bin/env python3
"""
簡化測試版啟動檔案 - 只啟動必要組件測試 odometry 和 RViz
使用: ros2 launch jetbot_ros jetbot_test.launch.py
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

        # ========== Static TF: map -> odom ==========
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),

        # ========== Motor Node ==========
        Node(
            package='jetbot_ros',
            executable='motors_node',
            name='motors_node',
            output='screen',
            respawn=False,  # 如果崩潰不重啟，方便看錯誤訊息
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

        # ========== Joint State Publisher ==========
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
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
