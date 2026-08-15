from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(
        get_package_share_directory('robot_localization_bringup')
    )
    ekf_config = package_share / 'config' / 'ekf.yaml'

    imu_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_link',
        output='screen',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.11',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link',
        ],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[str(ekf_config)],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
        ],
    )

    return LaunchDescription([
        imu_static_tf,
        ekf_node,
    ])
