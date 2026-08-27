from setuptools import find_packages, setup

package_name = 'landmark_geometry'

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
    description='Single source of truth for OP3 field landmark geometry + camera '
                'projection (world<->pixel, pixel->ground).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
