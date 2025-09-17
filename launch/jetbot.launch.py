#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # 啟動參數
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='是否啟用除錯模式顯示影像'
    )
    
    obstacle_threshold_arg = DeclareLaunchArgument(
        'obstacle_threshold',
        default_value='2000',
        description='障礙物檢測閾值'
    )
    
    motor_speed_arg = DeclareLaunchArgument(
        'motor_speed',
        default_value='120',
        description='馬達旋轉速度 (0-255)'
    )
    
    camera_mode_arg = DeclareLaunchArgument(
        'camera_mode',
        default_value='stereo',
        description='相機模式: single 或 stereo'
    )
    
    return LaunchDescription([
        # 宣告參數
        debug_mode_arg,
        obstacle_threshold_arg,
        motor_speed_arg,
        camera_mode_arg,
        
        LogInfo(msg="=== 啟動 JetBot 障礙物檢測系統 ==="),
        
        Node(
            package='jetbot_ros',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'width': 640,
                'height': 480,
                'framerate': 30,
                'debug_mode': LaunchConfiguration('debug_mode')
            }]
        ),

        Node(
            package='jetbot_ros',
            executable='motor_node',
            name='motor',
            output='screen',
            parameters=[{
                'rotation_speed': LaunchConfiguration('motor_speed'),
                'i2c_bus': 7,
                'left_motor_id': 1,
                'right_motor_id': 2
            }]
        ),
        
        LogInfo(msg="所有節點已啟動，系統運行中...")
])