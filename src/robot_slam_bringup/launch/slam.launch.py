from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('robot_slam_bringup'))
    slam_toolbox_share = Path(get_package_share_directory('slam_toolbox'))

    slam_config = package_share / 'config' / 'slam_toolbox.yaml'
    use_sim_time = LaunchConfiguration('use_sim_time')

    laser_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        output='screen',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.22',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '-1.57079632679',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser',
        ],
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(slam_toolbox_share / 'launch' / 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': str(slam_config),
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'use_lifecycle_manager': 'false',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use ROS simulation time.',
        ),
        laser_static_tf,
        slam_toolbox,
    ])
