import os
from glob import glob
from setuptools import setup

package_name = 'jetbot_ros'

# 安全的 glob 函數
def safe_glob(pattern):
    try:
        result = glob(pattern)
        return [f for f in result if os.path.isfile(f)]
    except:
        return []

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch 檔案
        (os.path.join('share', package_name, 'launch'), 
         safe_glob('launch/*.py')),
        # URDF 檔案
        (os.path.join('share', package_name, 'urdf'), 
         safe_glob('urdf/*.urdf') + safe_glob('urdf/*.xacro')),
        # RViz 配置檔
        (os.path.join('share', package_name, 'rviz'), 
         safe_glob('rviz/*.rviz')),
        # 相機校正檔案 - 左
        (os.path.join('share', package_name, 'config/camera_calibration/left'), 
         safe_glob('config/camera_calibration/left/*.yaml')),
        # 相機校正檔案 - 右
        (os.path.join('share', package_name, 'config/camera_calibration/right'), 
         safe_glob('config/camera_calibration/right/*.yaml')),
        (os.path.join('share', package_name, 'config/camera_calibration/stereo'), 
         safe_glob('config/camera_calibration/stereo/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='your.email@example.com',
    description='JetBot ROS2 package with stereo vision and SLAM for Jetson Orin Nano',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motors_node = jetbot_ros.motors_node:main',
            'teleop_keyboard = jetbot_ros.teleop_keyboard:main',
            'csi_camera_node = jetbot_ros.csi_camera_node:main',
        ],
    },
)