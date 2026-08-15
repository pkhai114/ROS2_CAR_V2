from setuptools import setup
import os
from glob import glob

package_name = 'robot_imu_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='t9',
    maintainer_email='t9@example.com',
    description='ROS2 driver for IM900/IM948-compatible IMU over USB-UART',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_driver_node = robot_imu_driver.imu_driver_node:main',
	    'imu_tf_broadcaster = robot_imu_driver.imu_tf_broadcaster:main',
        ],
    },
)
