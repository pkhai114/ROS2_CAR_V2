from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def include_launch(package_name, launch_file, launch_arguments=None):
    package_share = Path(get_package_share_directory(package_name))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / 'launch' / launch_file)
        ),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    return LaunchDescription([
        include_launch('robot_imu_driver', 'imu.launch.py'),
        include_launch('robot_lidar_bringup', 'lidar.launch.py'),
        include_launch('robot_localization_bringup', 'ekf.launch.py'),
        include_launch(
            'robot_slam_bringup',
            'slam.launch.py',
            {'use_sim_time': 'false'},
        ),
    ])
