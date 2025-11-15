from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('jetbot_ros'), 
        'config'
    )
    
    # 左相機節點 - 使用命名空間
    camera_left = Node(
        package='gscam',
        executable='gscam_node',
        name='gscam_node',  # 改成相同名稱
        namespace='camera_left',  # 加這行!
        output='screen',
        parameters=[os.path.join(config_dir, 'camera_left.yaml')]
    )
    
    # 右相機節點 - 使用命名空間
    camera_right = Node(
        package='gscam',
        executable='gscam_node',
        name='gscam_node',  # 改成相同名稱
        namespace='camera_right',  # 加這行!
        output='screen',
        parameters=[os.path.join(config_dir, 'camera_right.yaml')]
    )
    
    return LaunchDescription([
        camera_left,
        TimerAction(
            period=2.0,
            actions=[camera_right]
        )
    ])