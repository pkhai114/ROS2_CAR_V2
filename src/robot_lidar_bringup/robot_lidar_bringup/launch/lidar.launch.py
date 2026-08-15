from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('frame_id')
    scan_mode = LaunchConfiguration('scan_mode')
    min_distance = LaunchConfiguration('min_distance')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB_lidar',
            description='Serial port for RPLIDAR A1'
        ),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value='115200',
            description='Serial baudrate for RPLIDAR A1'
        ),

        DeclareLaunchArgument(
            'frame_id',
            default_value='laser',
            description='Laser frame ID'
        ),

        DeclareLaunchArgument(
            'scan_mode',
            default_value='Sensitivity',
            description='Scan mode for RPLIDAR A1'
        ),

        DeclareLaunchArgument(
            'min_distance',
            default_value='0.30',
            description='Discard scan ranges below this distance in metres'
        ),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': serial_port,
                'serial_baudrate': ParameterValue(
                    serial_baudrate,
                    value_type=int,
                ),
                'frame_id': frame_id,
                'scan_mode': scan_mode,
                'inverted': False,
                'angle_compensate': True,
            }],
            remappings=[
                ('scan', '/lidar/scan'),
            ],
        ),

        Node(
            package='robot_lidar_bringup',
            executable='scan_range_filter',
            name='scan_range_filter',
            output='screen',
            parameters=[{
                'input_topic': '/lidar/scan',
                'output_topic': '/scan/filtered',
                'min_distance': ParameterValue(
                    min_distance,
                    value_type=float,
                ),
            }]
        ),
    ])
