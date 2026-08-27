import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'landmark_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Muhammad Miftah',
    maintainer_email='muhammadmiftah13070@gmail.com',
    description='YOLOv8n field-landmark detector toolkit + ROS2 inference node.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'landmark_detector_node = landmark_detector.infer_node:main',
            'landmark_projector = landmark_detector.landmark_projector:main',
            'line_heading_node = landmark_detector.line_heading_node:main',
        ],
    },
)
