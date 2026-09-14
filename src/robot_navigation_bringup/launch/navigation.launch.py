#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_launch_description():
    package_dir = get_package_share_directory(
        'robot_navigation_bringup'
    )

    nav2_bringup_dir = get_package_share_directory(
        'nav2_bringup'
    )

    default_map = os.path.join(
        package_dir,
        'maps',
        'robot_map.yaml'
    )

    default_params_file = os.path.join(
        package_dir,
        'config',
        'nav2_params.yaml'
    )

    nav2_bringup_launch = os.path.join(
        nav2_bringup_dir,
        'launch',
        'bringup_launch.py'
    )

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    respawn = LaunchConfiguration('respawn')
    log_level = LaunchConfiguration('log_level')

    nav2_group = GroupAction([
        # Tất cả output cmd_vel của Nav2 (velocity_smoother và behavior_server)
        # đi vào một nhánh riêng. Chỉ velocity_arbiter được phát /cmd_vel.
                # Controller -> đầu vào velocity_smoother
        SetRemap(
            src='controller_server:cmd_vel',
            dst='/cmd_vel_nav',
        ),

        # Đầu vào velocity_smoother
        SetRemap(
            src='velocity_smoother:cmd_vel',
            dst='/cmd_vel_nav',
        ),

        # Đầu ra velocity_smoother -> arbiter
        SetRemap(
            src='velocity_smoother:cmd_vel_smoothed',
            dst='/cmd_vel_nav_output',
        ),

        # Vận tốc từ các behavior -> arbiter
        SetRemap(
            src='behavior_server:cmd_vel',
            dst='/cmd_vel_nav_output',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup_launch),
            launch_arguments={
                'namespace': '',
                'use_namespace': 'False',
                'slam': 'False',
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': autostart,
                'use_composition': use_composition,
                'respawn': respawn,
                'log_level': log_level,
            }.items()
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Duong dan den file YAML cua ban do'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Duong dan den file tham so Nav2'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Dung thoi gian mo phong'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='True',
            description='Tu dong kich hoat cac lifecycle node'
        ),
        DeclareLaunchArgument(
            'use_composition',
            default_value='False',
            description='Chay Nav2 bang cac node rieng'
        ),
        DeclareLaunchArgument(
            'respawn',
            default_value='False',
            description='Tu khoi dong lai node khi bi dung'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Muc log cua Nav2'
        ),
        nav2_group,
    ])
