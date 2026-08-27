import os
from glob import glob
from setuptools import setup

package_name = 'landmark_dataset_gen'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Muhammad Miftah',
    maintainer_email='muhammadmiftah13070@gmail.com',
    description='Auto-labeled soccer-field landmark dataset generator for YOLO '
                '(Webots ground-truth driven).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'landmark_dataset_capture = '
            'landmark_dataset_gen.landmark_dataset_capture:main',
            'landmark_dataset_sampler = '
            'landmark_dataset_gen.landmark_dataset_sampler:main',
        ],
    },
)
