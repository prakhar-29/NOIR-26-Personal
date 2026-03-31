from setuptools import setup
import os
from glob import glob

package_name = 'openvino_cone_detection'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        # Install model files
        (os.path.join('share', package_name, 'models'), 
            glob('models/*')),
        # Install config files (if any)
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Optimized OpenVINO-based cone detection for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'openvino_cone_detection_node = openvino_cone_detection.openvino_cone_detection_node:main',
        ],
    },
)
