from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'jetbot_ros'

def generate_data_files(dirs=['launch']):
    """
    Generate recursive list of data files, without listing directories in the output.
    """
    data_files = []
    for dir_path in dirs:
        if os.path.exists(dir_path):
            for path, _, files in os.walk(dir_path):
                install_dir = os.path.join('share', package_name, path)
                file_list = [os.path.join(path, f) for f in files if not f.startswith('.')]
                if file_list:
                    data_files.append((install_dir, file_list))
    return data_files

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test', 'build']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        
        # URDF files - 這是新加的！
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro') + glob('urdf/*.urdf')),
        
        # Camera calibration files
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/camera_calibration/left',
         glob('config/camera_calibration/left/*.yaml')),
        ('share/' + package_name + '/config/camera_calibration/right',
         glob('config/camera_calibration/right/*.yaml')),
        ('share/' + package_name + '/config/camera_calibration/stereo',
         glob('config/camera_calibration/stereo/*.yaml')),
        
        # RViz config
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
         
    ] + generate_data_files(['launch']),
    
    install_requires=[
        'setuptools',
        'rclpy',
        'geometry_msgs',
        'sensor_msgs', 
        'std_msgs',
        'cv_bridge',
        'Adafruit-MotorHAT',
        'opencv-python',
        'numpy',
        'Pillow',
    ],
    zip_safe=True,
    maintainer='jetbot_user',
    maintainer_email='user@example.com',
    description='ROS2 nodes for JetBot with dual camera obstacle avoidance',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = jetbot_ros.camera_node:main',
            'motors_node = jetbot_ros.motors_node:main',
            'teleop_keyboard = jetbot_ros.teleop_keyboard:main',
            'data_collection = jetbot_ros.data_collection:main',
            'train_navigation = jetbot_ros.dnn.train:main',
            'reshape_model = jetbot_ros.dnn.reshape_model:main',
        ],
    },
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'Programming Language :: Python :: 3',
        'Topic :: Scientific/Engineering :: Robotics',
    ],
    python_requires='>=3.8',
)