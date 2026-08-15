from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_imu_driver')
    config_file = os.path.join(pkg_share, 'config', 'imu.yaml')

    imu_node = Node(
        package='robot_imu_driver',
        executable='imu_driver_node',
        name='imu_driver_node',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        imu_node,
    ])
