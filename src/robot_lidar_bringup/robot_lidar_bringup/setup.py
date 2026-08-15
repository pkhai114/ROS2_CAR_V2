from setuptools import setup
import os
from glob import glob

package_name = 'robot_lidar_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='t9',
    maintainer_email='t9@example.com',
    description='Bringup package for RPLIDAR A1 on robot arm car project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_range_filter = robot_lidar_bringup.scan_range_filter:main',
        ],
    },
)
