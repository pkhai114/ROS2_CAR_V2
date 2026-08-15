import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory(
        'robot_localization_bringup'
    )
    params_file = os.path.join(package_share, 'config', 'amcl.yaml')

    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser(
            '~/ros2_ws/maps/robot_map.yaml'
        ),
        description='Absolute path to the saved map YAML file.',
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock when true.',
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {
                'yaml_filename': map_file,
                'use_sim_time': use_sim_time,
            },
        ],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        declare_map,
        declare_use_sim_time,
        map_server,
        amcl,
        lifecycle_manager,
    ])
