from setuptools import setup, find_packages
from glob import glob
from itertools import chain
import os

package_name = 'jetbot_ros'

def generate_data_files(dirs=['launch']):
    """
    Generate recursive list of data files, without listing directories in the output.
    Removed gazebo directories as they are no longer needed for real hardware deployment.
    """
    data_files = []
    for dir_path in dirs:
        if os.path.exists(dir_path):
            for path, _, files in os.walk(dir_path):
                install_dir = os.path.join('share', package_name, path)
                file_list = [os.path.join(path, f) for f in files if not f.startswith('.')]
                if file_list:  # Only add if there are files to install
                    data_files.append((install_dir, file_list))
    return data_files

def get_data_files_recursive(directory, target_dir=None):
    """Helper function to get files from a directory recursively"""
    if not os.path.exists(directory):
        return []
    
    if target_dir is None:
        target_dir = directory
    
    files = []
    for root, _, filenames in os.walk(directory):
        for filename in filenames:
            if not filename.startswith('.'):
                files.append(os.path.join(root, filename))
    
    if files:
        return [(os.path.join('share', package_name, target_dir), files)]
    return []

setup(
    name=package_name,
    version='0.1.0',  # Updated version to reflect modifications
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Existing data directory (contains models and datasets)
        ('share/' + package_name + '/data/models', 
         glob('data/models/*.pth') if os.path.exists('data/models') else []),
        ('share/' + package_name + '/data/datasets', 
         glob('data/datasets/**/*', recursive=True) if os.path.exists('data/datasets') else []),
        
        # Configuration files (create if needed for your custom parameters)
        ('share/' + package_name + '/config', 
         glob('config/*.yaml') if os.path.exists('config') else []),
         
        # Documentation
        ('share/' + package_name + '/docs', 
         glob('docs/*.md') if os.path.exists('docs') else []),
         
    ] + generate_data_files(),
    install_requires=[
        'setuptools',
        # ROS2 dependencies
        'rclpy',
        'geometry_msgs',
        'sensor_msgs', 
        'std_msgs',
        'nav_msgs',
        'cv_bridge',
        'image_transport',
        # Hardware control dependencies
        'adafruit-circuitpython-pca9685',
        'adafruit-circuitpython-motor',
        'board',
        'busio',
        # AI and image processing
        'torch',
        'torchvision', 
        'opencv-python',
        'numpy',
        'Pillow',
        'scikit-image',
        # Additional utilities
        'pynput',  # For keyboard input
        'flask',   # For future web interface
    ],
    zip_safe=True,
    maintainer='Dustin Franklin',
    maintainer_email='dustinf@nvidia.com',
    description='ROS2 nodes for JetBot - Real Hardware Version with dual camera navigation support',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Core functionality (keep existing)
            'teleop_keyboard = jetbot_ros.teleop_keyboard:main',
            'data_collection = jetbot_ros.data_collection:main',
            'nav_model = jetbot_ros.nav_model:main',  # Keep for learning/testing
            
            # Hardware control (only Adafruit/NVIDIA version)
            'motors_nvidia = jetbot_ros.motors_nvidia:main',
            
            # Display control
            'oled_ssd1306 = jetbot_ros.oled_ssd1306:main',
            
            # Future dual camera and navigation nodes
            # (Uncomment when you implement them)
            # 'dual_camera_node = jetbot_ros.dual_camera_node:main',
            # 'stereo_vision_node = jetbot_ros.stereo_vision_node:main',
            # 'obstacle_avoidance_stereo = jetbot_ros.obstacle_avoidance_stereo:main',
            # 'slam_navigation_node = jetbot_ros.slam_navigation_node:main',
            # 'object_detection_node = jetbot_ros.object_detection_node:main',
            # 'voice_command_node = jetbot_ros.voice_command_node:main',
            # 'mobile_interface_node = jetbot_ros.mobile_interface_node:main',
            # 'task_planning_node = jetbot_ros.task_planning_node:main',
        ],
    },
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'Intended Audience :: Education',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Topic :: Scientific/Engineering :: Robotics',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Topic :: Software Development :: Libraries :: Python Modules',
    ],
    python_requires='>=3.8',
    keywords='ros2 robotics jetbot nvidia jetson-nano computer-vision navigation stereo-vision voice-control',
)