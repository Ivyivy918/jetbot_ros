#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import LogInfo
import os
from ament_index_python.packages import get_package_share_directory

os.environ['DISPLAY'] = ':0'

def generate_launch_description():
    pkg_share = get_package_share_directory('jetbot_ros')

    # 先找 urdf，支援 .urdf 和 .urdf.xacro 兩種副檔名
    urdf_path_plain = os.path.join(pkg_share, 'urdf', 'jetbot.urdf')
    urdf_path_xacro = os.path.join(pkg_share, 'urdf', 'jetbot.urdf.xacro')

    camera_left_yaml  = os.path.join(pkg_share, 'config', 'left.yaml')
    camera_right_yaml = os.path.join(pkg_share, 'config', 'right.yaml')

    nodes = []

    # ═══════════════════════════════════════════
    # 機器人狀態發布（robot_state_publisher）
    # ═══════════════════════════════════════════
    if os.path.exists(urdf_path_plain):
        # 純 URDF：直接讀檔，不用 xacro
        with open(urdf_path_plain, 'r') as f:
            robot_desc = f.read()
        nodes.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': robot_desc,
                    'use_sim_time': False
                }]
            )
        )
    elif os.path.exists(urdf_path_xacro):
        # xacro 格式
        nodes.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': Command(['xacro ', urdf_path_xacro]),
                    'use_sim_time': False
                }]
            )
        )
    else:
        nodes.append(LogInfo(msg='[WARNING] URDF 找不到，robot_state_publisher 未啟動'))

    # ═══════════════════════════════════════════
    # Joint State Publisher（輪子 TF 必要）
    # 沒有這個 → left_wheel / right_wheel TF 斷掉
    # → RViz 報 "No transform from [wheel_left] to [base_footprint]"
    # ═══════════════════════════════════════════
    nodes.append(
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        )
    )

    # ═══════════════════════════════════════════
    # 雙相機擷取節點
    # ═══════════════════════════════════════════
    nodes.append(
        Node(
            package='jetbot_ros',
            executable='camera_node',
            name='stereo_pointcloud_node',
            output='screen',
            additional_env={'DISPLAY': ':0'}
        )
    )

    # ═══════════════════════════════════════════
    # 深度估計節點
    # ═══════════════════════════════════════════
    nodes.append(
        Node(
            package='jetbot_ros',
            executable='stereo_depth_node',
            name='stereo_depth_node',
            output='screen',
            parameters=[{
                'left_camera_config':  camera_left_yaml,
                'right_camera_config': camera_right_yaml,
                'baseline':       0.11,
                'min_disparity':  0,
                'num_disparities': 64,
                'block_size':     15
            }]
        )
    )

    # ═══════════════════════════════════════════
    # 馬達控制節點
    # ═══════════════════════════════════════════
    nodes.append(
        Node(
            package='jetbot_ros',
            executable='motors_node',
            name='motors_node',
            output='screen'
        )
    )

    # ═══════════════════════════════════════════
    # RTAB-Map SLAM
    # ═══════════════════════════════════════════
    nodes.append(
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'subscribe_stereo':           True,
                'subscribe_depth':            False,
                'frame_id':                   'base_link',
                'map_frame_id':               'map',
                'odom_frame_id':              'odom',
                'publish_tf':                 True,
                'approx_sync':                True,
                'queue_size':                 30,
                'Rtabmap/DetectionRate':      '2.0',
                'RGBD/AngularUpdate':         '0.05',
                'RGBD/LinearUpdate':          '0.05',
                'RGBD/OptimizeFromGraphEnd':  'false',
                'Optimizer/Strategy':         '0',
                'Mem/IncrementalMemory':      'true',
                'Mem/InitWMWithAllNodes':     'false',
                'Stereo/MaxDisparity':        '256',
                'Stereo/MinDisparity':        '1',
                'Stereo/OpticalFlow':         'true',
                'Vis/FeatureType':            '6',
                'Vis/MaxFeatures':            '1000',
                'Vis/MinInliers':             '10',
                'Vis/InlierDistance':         '0.1',
                'Kp/DetectorStrategy':        '6',
                'Kp/MaxFeatures':             '400',
            }],
            remappings=[
                ('left/image_rect',   '/camera_left/image_raw'),
                ('right/image_rect',  '/camera_right/image_raw'),
                ('left/camera_info',  '/camera_left/camera_info'),
                ('right/camera_info', '/camera_right/camera_info'),
            ],
            arguments=['--delete_db_on_start']
        )
    )

    # ═══════════════════════════════════════════
    # RViz2
    # ═══════════════════════════════════════════
    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            additional_env={'DISPLAY': ':0'}
        )
    )

    return LaunchDescription(nodes)