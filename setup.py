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
            for path, _, files in os.walk(dir_path):  # 修正這裡
                install_dir = os.path.join('share', package_name, path)
                file_list = [os.path.join(path, f) for f in files if not f.startswith('.')]
                if file_list:  # Only add if there are files to install
                    data_files.append((install_dir, file_list))
    return data_files

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Configuration files
        ('share/' + package_name + '/config', 
         glob('config/*.yaml') if os.path.exists('config') else []),
         
        # Launch files
    ] + generate_data_files(),
    
    install_requires=[
        'setuptools',
        # ROS2 dependencies
        'rclpy',
        'geometry_msgs',
        'sensor_msgs', 
        'std_msgs',
        'cv_bridge',
        # Hardware control dependencies
        'Adafruit-MotorHAT',
        # Image processing
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
            # 根據你實際的檔案匹配
            'camera_node = jetbot_ros.camera_node:main',
            'motor_node = jetbot_ros.motors_node:main',
            
            
            # 其他現有節點 (用 # 註解而不是 ''')
            # 'stereo_vision_node = jetbot_ros.stereo_vision:main', 
            # 'data_collection = jetbot_ros.data_collection:main',
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