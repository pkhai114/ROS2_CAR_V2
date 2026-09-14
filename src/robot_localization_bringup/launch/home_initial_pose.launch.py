import os

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory(
        'robot_localization_bringup'
    )

    default_params_file = os.path.join(
        package_share,
        'config',
        'home_initial_pose.yaml',
    )

    params_file = LaunchConfiguration('params_file')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='File cấu hình HOME Initial Pose.',
    )

    home_initial_pose_node = Node(
        package='robot_localization_bringup',
        executable='home_initial_pose_node.py',
        name='home_initial_pose_node',
        output='screen',
        parameters=[params_file],
        respawn=False,
    )

    return LaunchDescription([
        declare_params_file,
        home_initial_pose_node,
    ])