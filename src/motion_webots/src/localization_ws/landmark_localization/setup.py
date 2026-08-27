from setuptools import find_packages, setup

package_name = 'landmark_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Muhammad Miftah',
    maintainer_email='muhammadmiftah13070@gmail.com',
    description='Geometric landmark localization backend (CLAP/ILM MHL) + '
                'GT fake_detector + offline sensitivity sweep.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_detector = landmark_localization.fake_detector:main',
            'geometric_pose_node = '
            'landmark_localization.geometric_pose_node:main',
            'gaze_localization_node = '
            'landmark_localization.gaze_localization_node:main',
            'degrade_relay = landmark_localization.degrade_relay:main',
        ],
    },
)
