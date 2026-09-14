#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory(
        'robot_navigation_bringup'
    )

    default_station_file = os.path.join(
        package_share,
        'config',
        'stations.yaml',
    )

    station_file = LaunchConfiguration('station_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'station_file',
            default_value=default_station_file,
            description='Đường dẫn tới file cấu hình HOME và các trạm.',
        ),

        Node(
            package='robot_navigation_bringup',
            executable='station_visualizer_node.py',
            name='station_visualizer_node',
            output='screen',
            parameters=[{
                'station_file': station_file,
                'marker_topic': '/station_markers',
                'status_topic': '/station/status',
                'republish_period_sec': 2.0,
                'use_sim_time': False,
            }],
        ),

        Node(
            package='robot_navigation_bringup',
            executable='station_manager_node.py',
            name='station_manager_node',
            output='screen',
            parameters=[{
                'station_file': station_file,
                'command_topic': '/station/command',
                'status_topic': '/station/status',
                'current_topic': '/station/current',
                'distance_remaining_topic': (
                    '/station/distance_remaining'
                ),
                'navigate_action': '/navigate_to_pose',
                'action_server_timeout_sec': 2.0,
                'use_sim_time': False,
            }],
        ),
    ])
