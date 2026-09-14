#!/usr/bin/env python3

import math
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String


class StationManagerNode(Node):
    """Send Nav2 goals to HOME or to a station staging pose."""

    def __init__(self):
        super().__init__('station_manager_node')

        self.declare_parameter('station_file', '')
        self.declare_parameter('command_topic', '/station/command')
        self.declare_parameter('status_topic', '/station/status')
        self.declare_parameter('current_topic', '/station/current')
        self.declare_parameter(
            'distance_remaining_topic',
            '/station/distance_remaining',
        )
        self.declare_parameter('navigate_action', '/navigate_to_pose')
        self.declare_parameter('action_server_timeout_sec', 2.0)

        station_file = str(self.get_parameter('station_file').value)
        command_topic = str(self.get_parameter('command_topic').value)
        status_topic = str(self.get_parameter('status_topic').value)
        current_topic = str(self.get_parameter('current_topic').value)
        distance_topic = str(
            self.get_parameter('distance_remaining_topic').value
        )
        navigate_action = str(
            self.get_parameter('navigate_action').value
        )
        self.action_server_timeout = float(
            self.get_parameter('action_server_timeout_sec').value
        )

        if not station_file:
            raise ValueError('Tham số station_file không được để trống.')

        self.frame_id, self.stations = self.load_station_file(station_file)

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.status_publisher = self.create_publisher(
            String,
            status_topic,
            state_qos,
        )
        self.current_publisher = self.create_publisher(
            String,
            current_topic,
            state_qos,
        )
        self.distance_publisher = self.create_publisher(
            Float32,
            distance_topic,
            10,
        )

        self.command_subscription = self.create_subscription(
            String,
            command_topic,
            self.command_callback,
            10,
        )

        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            navigate_action,
        )

        self.goal_handle = None
        self.pending_station = ''
        self.active_station = ''
        self.cancel_requested = False

        self.publish_status('IDLE', '')
        self.publish_current('')

        self.get_logger().info(
            f'Đã nạp {len(self.stations)} vị trí từ {station_file}'
        )
        self.get_logger().info(
            f'Nhận lệnh tại {command_topic}; Nav2 action={navigate_action}'
        )

    @staticmethod
    def require_number(container, key, context):
        if key not in container:
            raise ValueError(f'Thiếu {context}.{key}')

        value = float(container[key])
        if not math.isfinite(value):
            raise ValueError(f'{context}.{key} phải là số hữu hạn.')
        return value

    def load_pose(self, container, context):
        return {
            'x': self.require_number(container, 'x', context),
            'y': self.require_number(container, 'y', context),
            'yaw': self.require_number(container, 'yaw', context),
        }

    def load_station_file(self, station_file):
        path = Path(station_file).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f'Không tìm thấy station_file: {path}')

        with path.open('r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream)

        if not isinstance(data, dict):
            raise ValueError('stations.yaml phải chứa một YAML mapping.')

        system = data.get('station_system', {})
        frame_id = str(system.get('frame_id', 'map'))
        raw_stations = data.get('stations')

        if not isinstance(raw_stations, dict) or not raw_stations:
            raise ValueError('stations.yaml không có mục stations hợp lệ.')

        stations = {}
        for key, raw in raw_stations.items():
            if not isinstance(raw, dict):
                raise ValueError(f'Cấu hình {key} không hợp lệ.')

            station_type = str(raw.get('type', 'station')).strip().lower()

            if station_type == 'home':
                target_pose = self.load_pose(raw['pose'], f'{key}.pose')
            else:
                # Phiên bản này chỉ dùng Nav2 tới staging_pose.
                # dock_pose sẽ dành cho docking_controller_node.py.
                target_pose = self.load_pose(
                    raw['staging_pose'],
                    f'{key}.staging_pose',
                )

            stations[str(key)] = {
                'display_name': str(raw.get('display_name', key)),
                'type': station_type,
                'target_pose': target_pose,
            }

        return frame_id, stations

    def publish_status(self, state, station_key):
        msg = String()
        msg.data = f'{state}:{station_key}' if station_key else state
        self.status_publisher.publish(msg)

    def publish_current(self, station_key):
        msg = String()
        msg.data = station_key
        self.current_publisher.publish(msg)

    def normalize_command(self, command):
        normalized = command.strip().lower().replace('-', '_').replace(' ', '_')

        aliases = {
            '0': 'home',
            'home': 'home',
            '1': 'station_1',
            'station1': 'station_1',
            'station_1': 'station_1',
            'tram1': 'station_1',
            'tram_1': 'station_1',
            '2': 'station_2',
            'station2': 'station_2',
            'station_2': 'station_2',
            'tram2': 'station_2',
            'tram_2': 'station_2',
        }
        return aliases.get(normalized, normalized)

    def command_callback(self, msg):
        raw_command = msg.data.strip()
        if not raw_command:
            self.get_logger().warning('Bỏ qua lệnh trạm rỗng.')
            return

        if raw_command.lower() in ('cancel', 'stop'):
            self.cancel_active_goal()
            return

        station_key = self.normalize_command(raw_command)

        if station_key not in self.stations:
            self.get_logger().error(
                f'Không tồn tại trạm "{raw_command}". '
                f'Các lựa chọn: {list(self.stations.keys())}'
            )
            self.publish_status('INVALID_COMMAND', station_key)
            return

        if self.goal_handle is not None or self.pending_station:
            self.get_logger().warning(
                f'Đang xử lý {self.active_station or self.pending_station}; '
                'hãy gửi lệnh cancel trước khi chọn trạm khác.'
            )
            self.publish_status(
                'BUSY',
                self.active_station or self.pending_station,
            )
            return

        self.send_navigation_goal(station_key)

    def send_navigation_goal(self, station_key):
        if not self.navigate_client.wait_for_server(
            timeout_sec=self.action_server_timeout
        ):
            self.get_logger().error(
                'Không tìm thấy Nav2 action server /navigate_to_pose.'
            )
            self.publish_status('NAV2_UNAVAILABLE', station_key)
            return

        station = self.stations[station_key]
        pose = station['target_pose']

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = pose['x']
        goal.pose.pose.position.y = pose['y']
        goal.pose.pose.position.z = 0.0

        half_yaw = pose['yaw'] / 2.0
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = math.sin(half_yaw)
        goal.pose.pose.orientation.w = math.cos(half_yaw)

        self.pending_station = station_key
        self.cancel_requested = False
        self.publish_status('SENDING_GOAL', station_key)

        self.get_logger().info(
            f'Gửi Nav2 goal tới {station["display_name"]}: '
            f'x={pose["x"]:.3f}, y={pose["y"]:.3f}, '
            f'yaw={pose["yaw"]:.3f} rad'
        )

        send_future = self.navigate_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        station_key = self.pending_station

        try:
            goal_handle = future.result()
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f'Lỗi gửi Nav2 goal: {error}')
            self.pending_station = ''
            self.cancel_requested = False
            self.publish_status('FAILED', station_key)
            return

        if not goal_handle.accepted:
            self.get_logger().error(
                f'Nav2 từ chối goal tới {station_key}.'
            )
            self.pending_station = ''
            self.cancel_requested = False
            self.publish_status('REJECTED', station_key)
            return

        self.goal_handle = goal_handle
        self.active_station = station_key
        self.pending_station = ''
        self.publish_current(station_key)
        self.publish_status('NAVIGATING', station_key)

        self.get_logger().info(
            f'Nav2 đã chấp nhận goal tới {station_key}.'
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

        if self.cancel_requested:
            self.get_logger().warning(
                f'Goal {station_key} vừa được chấp nhận nhưng đã có yêu cầu hủy.'
            )
            self.goal_handle.cancel_goal_async()

    def feedback_callback(self, feedback_msg):
        distance = float(feedback_msg.feedback.distance_remaining)
        msg = Float32()
        msg.data = distance
        self.distance_publisher.publish(msg)

    def navigation_result_callback(self, future):
        station_key = self.active_station

        try:
            wrapped_result = future.result()
            status = wrapped_result.status
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f'Lỗi nhận kết quả Nav2: {error}')
            status = GoalStatus.STATUS_ABORTED

        if status == GoalStatus.STATUS_SUCCEEDED:
            if self.stations[station_key]['type'] == 'home':
                final_state = 'AT_HOME'
            else:
                final_state = 'AT_STAGING'

            self.get_logger().info(
                f'Đã đến {station_key}: {final_state}'
            )
        elif status == GoalStatus.STATUS_CANCELED:
            final_state = 'CANCELED'
            self.get_logger().warning(f'Goal {station_key} đã bị hủy.')
        else:
            final_state = 'FAILED'
            self.get_logger().error(
                f'Không thể đến {station_key}; Nav2 status={status}.'
            )

        self.publish_status(final_state, station_key)
        if final_state in ('CANCELED', 'FAILED'):
            self.publish_current('')
        self.goal_handle = None
        self.active_station = ''
        self.cancel_requested = False

    def cancel_active_goal(self):
        if self.pending_station and self.goal_handle is None:
            self.cancel_requested = True
            self.publish_status('CANCELLING', self.pending_station)
            self.get_logger().warning(
                f'Đã ghi nhận yêu cầu hủy goal tới {self.pending_station}.'
            )
            return

        if self.goal_handle is None:
            self.get_logger().warning('Không có Nav2 goal đang chạy để hủy.')
            self.publish_status('IDLE', '')
            return

        station_key = self.active_station
        self.publish_status('CANCELLING', station_key)
        self.get_logger().warning(f'Đang hủy goal tới {station_key}...')
        self.goal_handle.cancel_goal_async()


def main(args=None):
    rclpy.init(args=args)
    node = StationManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
