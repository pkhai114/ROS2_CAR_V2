#!/usr/bin/env python3
"""Station AUTO only: run Nav2/localization/sensors separately, exactly once."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    docking_share = get_package_share_directory('robot_docking')
    navigation_share = get_package_share_directory('robot_navigation_bringup')
    station_file = LaunchConfiguration('station_file')
    params_file = LaunchConfiguration('params_file')
    manager_params_file = LaunchConfiguration('manager_params_file')
    sim = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    # No automatic respawn/resumption of movement. Peers stop on lost heartbeats.
    return LaunchDescription([
        DeclareLaunchArgument('station_file', default_value=os.path.join(
            navigation_share, 'config', 'stations.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            docking_share, 'config', 'docking_params.yaml')),
        DeclareLaunchArgument('manager_params_file', default_value=os.path.join(
            navigation_share, 'config', 'station_auto_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='robot_docking', executable='velocity_arbiter_node.py',
             name='velocity_arbiter', output='screen',
             parameters=[params_file, {'default_mode': 'HOLD',
                         'require_station_heartbeat': True,
                         'station_heartbeat_timeout': 1.0, 'use_sim_time': sim}]),
        Node(package='robot_docking', executable='docking_controller_node.py',
             name='docking_controller', output='screen',
             parameters=[params_file, {'station_file': station_file,
                         'managed_only': True, 'use_sim_time': sim}]),
        Node(package='robot_navigation_bringup', executable='station_auto_manager_node.py',
             name='station_manager_node', output='screen',
             parameters=[manager_params_file, {'station_file': station_file, 'use_sim_time': sim}]),
        Node(package='robot_navigation_bringup', executable='station_visualizer_node.py',
             name='station_visualizer_node', output='screen',
             parameters=[{'station_file': station_file, 'marker_topic': '/station_markers',
                          'status_topic': '/station/status', 'republish_period_sec': 2.0,
                          'use_sim_time': sim}]),
    ])
