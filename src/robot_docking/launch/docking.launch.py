#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    docking_share = get_package_share_directory('robot_docking')
    navigation_share = get_package_share_directory(
        'robot_navigation_bringup'
    )

    default_params_file = os.path.join(
        docking_share,
        'config',
        'docking_params.yaml',
    )
    default_station_file = os.path.join(
        navigation_share,
        'config',
        'stations.yaml',
    )

    params_file = LaunchConfiguration('params_file')
    station_file = LaunchConfiguration('station_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='File tham số cho docking và velocity arbiter.',
        ),
        DeclareLaunchArgument(
            'station_file',
            default_value=default_station_file,
            description='File stations.yaml đang dùng bởi hệ thống trạm.',
        ),
        Node(
            package='robot_docking',
            executable='velocity_arbiter_node.py',
            name='velocity_arbiter',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='robot_docking',
            executable='docking_controller_node.py',
            name='docking_controller',
            output='screen',
            parameters=[
                params_file,
                {'station_file': station_file},
            ],
        ),
    ])
