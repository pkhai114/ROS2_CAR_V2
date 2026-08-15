import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_robot_lidar_bringup = get_package_share_directory('robot_lidar_bringup')

    rviz_config_file = os.path.join(
        pkg_robot_lidar_bringup,
        'rviz',
        'lidar.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_lidar_view',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        rviz_node,
    ])
