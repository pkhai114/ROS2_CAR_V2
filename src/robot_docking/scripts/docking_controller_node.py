#!/usr/bin/env python3

"""Dock and undock a differential-drive robot on a short straight path."""

import hashlib
import json
import math
import time
import uuid
from collections import OrderedDict
from pathlib import Path
from typing import Optional

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSPresetProfiles,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformException, TransformListener


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z
        + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


class DockingControllerNode(Node):
    """State machine for straight-line docking and reverse undocking."""

    ACTIVE_STATES = {
        'WAIT_STOP',
        'ALIGN_START',
        'FOLLOW_PATH',
        'ALIGN_FINAL',
        'VERIFY_FINAL',
    }

    def __init__(self):
        super().__init__('docking_controller')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('station_file', ''),
                ('managed_only', False),
                ('global_frame', 'map'),
                ('base_frame', 'base_link'),
                ('scan_topic', '/scan/filtered'),
                ('odom_topic', '/odometry/filtered'),
                ('command_topic', '/docking/command'),
                ('status_topic', '/docking/status'),
                ('dock_velocity_topic', '/cmd_vel_dock'),
                ('arbiter_mode_topic', '/velocity_arbiter/mode'),
                ('arbiter_state_topic', '/velocity_arbiter/state'),
                ('control_frequency', 20.0),
                ('sensor_timeout', 0.50),
                ('tf_timeout', 0.50),
                ('arbiter_timeout', 1.00),
                ('operation_timeout', 30.0),
                ('stop_linear_threshold', 0.015),
                ('stop_angular_threshold', 0.035),
                ('stop_stable_time', 0.50),
                ('start_position_tolerance', 0.10),
                ('start_yaw_tolerance_deg', 15.0),
                ('alignment_max_angular_speed', 0.15),
                ('alignment_timeout', 8.0),
                ('undock_start_position_tolerance', 0.14),
                ('lookahead_distance', 0.10),
                ('forward_speed', 0.08),
                ('reverse_speed', 0.07),
                ('minimum_speed', 0.025),
                ('slowdown_distance', 0.18),
                ('linear_acceleration_limit', 0.20),
                ('angular_acceleration_limit', 0.60),
                ('heading_kp', 1.20),
                ('heading_kd', 0.12),
                ('derivative_filter_alpha', 0.30),
                ('yaw_kp', 1.40),
                ('maximum_angular_speed', 0.30),
                ('minimum_angular_speed', 0.055),
                ('drive_heading_limit_deg', 25.0),
                ('path_lateral_limit', 0.15),
                ('path_position_tolerance', 0.045),
                ('final_position_tolerance', 0.08),
                ('align_yaw_tolerance_deg', 3.0),
                ('final_yaw_tolerance_deg', 5.0),
                ('final_stable_time', 0.50),
                ('final_condition_timeout', 3.0),
                ('front_scan_angle', 1.57079632679),
                ('rear_scan_angle', -1.57079632679),
                ('scan_sector_half_width_deg', 10.0),
                ('target_front_distance', 0.37),
                ('front_distance_tolerance', 0.04),
                ('front_hard_stop_distance', 0.33),
                ('front_distance_assist_window', 0.15),
                ('rear_obstacle_stop_distance', 0.37),
            ],
        )

        self.global_frame = self.string_parameter('global_frame')
        self.base_frame = self.string_parameter('base_frame')
        station_file = self.string_parameter('station_file')
        if not station_file:
            raise ValueError('Tham số station_file không được để trống.')

        self.frame_id, self.stations = self.load_stations(station_file)
        self.station_hash = hashlib.sha256(Path(station_file).expanduser().read_bytes()).hexdigest()
        self.managed_only = bool(self.get_parameter('managed_only').value)
        if self.global_frame != self.frame_id:
            self.get_logger().warning(
                f'global_frame={self.global_frame}, nhưng stations.yaml dùng '
                f'{self.frame_id}; sử dụng {self.frame_id}.'
            )
            self.global_frame = self.frame_id

        self.control_frequency = self.positive_parameter('control_frequency')
        self.sensor_timeout = self.positive_parameter('sensor_timeout')
        self.tf_timeout = self.positive_parameter('tf_timeout')
        self.arbiter_timeout = self.positive_parameter('arbiter_timeout')
        self.operation_timeout = self.positive_parameter('operation_timeout')
        self.stop_linear_threshold = self.positive_parameter(
            'stop_linear_threshold'
        )
        self.stop_angular_threshold = self.positive_parameter(
            'stop_angular_threshold'
        )
        self.stop_stable_time = self.positive_parameter('stop_stable_time')
        self.start_position_tolerance = self.positive_parameter(
            'start_position_tolerance'
        )
        self.undock_start_position_tolerance = self.positive_parameter(
            'undock_start_position_tolerance'
        )
        self.lookahead_distance = self.positive_parameter(
            'lookahead_distance'
        )
        self.forward_speed = self.positive_parameter('forward_speed')
        self.reverse_speed = self.positive_parameter('reverse_speed')
        self.minimum_speed = self.positive_parameter('minimum_speed')
        self.slowdown_distance = self.positive_parameter('slowdown_distance')
        self.linear_acceleration_limit = self.positive_parameter(
            'linear_acceleration_limit'
        )
        self.angular_acceleration_limit = self.positive_parameter(
            'angular_acceleration_limit'
        )
        self.heading_kp = self.positive_parameter('heading_kp')
        self.heading_kd = float(self.get_parameter('heading_kd').value)
        self.derivative_filter_alpha = float(
            self.get_parameter('derivative_filter_alpha').value
        )
        self.yaw_kp = self.positive_parameter('yaw_kp')
        self.maximum_angular_speed = self.positive_parameter(
            'maximum_angular_speed'
        )
        self.minimum_angular_speed = self.positive_parameter(
            'minimum_angular_speed'
        )
        self.start_yaw_tolerance = math.radians(
            self.positive_parameter('start_yaw_tolerance_deg')
        )
        self.alignment_max_angular_speed = self.positive_parameter(
            'alignment_max_angular_speed'
        )
        self.alignment_timeout = self.positive_parameter('alignment_timeout')

        if not (
            self.minimum_angular_speed
            <= self.alignment_max_angular_speed
            <= self.maximum_angular_speed
        ):
            raise ValueError(
                'Cần minimum_angular_speed <= alignment_max_angular_speed '
                '<= maximum_angular_speed.'
            )
        self.drive_heading_limit = math.radians(
            self.positive_parameter('drive_heading_limit_deg')
        )
        self.path_lateral_limit = self.positive_parameter(
            'path_lateral_limit'
        )
        self.path_position_tolerance = self.positive_parameter(
            'path_position_tolerance'
        )
        self.final_position_tolerance = self.positive_parameter(
            'final_position_tolerance'
        )
        self.align_yaw_tolerance = math.radians(
            self.positive_parameter('align_yaw_tolerance_deg')
        )
        self.final_yaw_tolerance = math.radians(
            self.positive_parameter('final_yaw_tolerance_deg')
        )
        self.final_stable_time = self.positive_parameter('final_stable_time')
        self.final_condition_timeout = self.positive_parameter(
            'final_condition_timeout'
        )
        self.front_scan_angle = float(
            self.get_parameter('front_scan_angle').value
        )
        self.rear_scan_angle = float(
            self.get_parameter('rear_scan_angle').value
        )
        self.scan_sector_half_width = math.radians(
            self.positive_parameter('scan_sector_half_width_deg')
        )
        self.target_front_distance = self.positive_parameter(
            'target_front_distance'
        )
        self.front_distance_tolerance = self.positive_parameter(
            'front_distance_tolerance'
        )
        self.front_hard_stop_distance = self.positive_parameter(
            'front_hard_stop_distance'
        )
        self.front_distance_assist_window = self.positive_parameter(
            'front_distance_assist_window'
        )
        self.rear_obstacle_stop_distance = self.positive_parameter(
            'rear_obstacle_stop_distance'
        )

        if not 0.0 < self.derivative_filter_alpha <= 1.0:
            raise ValueError('derivative_filter_alpha phải trong (0, 1].')
        if self.minimum_speed > min(self.forward_speed, self.reverse_speed):
            raise ValueError('minimum_speed không được lớn hơn tốc độ chạy.')
        if self.front_hard_stop_distance >= self.target_front_distance:
            raise ValueError(
                'front_hard_stop_distance phải nhỏ hơn target_front_distance.'
            )

        scan_topic = self.string_parameter('scan_topic')
        odom_topic = self.string_parameter('odom_topic')
        command_topic = self.string_parameter('command_topic')
        status_topic = self.string_parameter('status_topic')
        velocity_topic = self.string_parameter('dock_velocity_topic')
        arbiter_mode_topic = self.string_parameter('arbiter_mode_topic')
        arbiter_state_topic = self.string_parameter('arbiter_state_topic')

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.velocity_publisher = self.create_publisher(
            Twist,
            velocity_topic,
            10,
        )
        self.mode_publisher = self.create_publisher(
            String,
            arbiter_mode_topic,
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            status_topic,
            state_qos,
        )
        self.sequence_publisher = self.create_publisher(String, '/docking/sequence_status', 10)
        self.create_subscription(String, '/docking/sequence_command', self.sequence_callback, 10)
        self.create_subscription(String, '/station/heartbeat', self.manager_heartbeat, 10)
        self.distance_publisher = self.create_publisher(
            Float32,
            '/docking/distance_remaining',
            10,
        )
        self.heading_publisher = self.create_publisher(
            Float32,
            '/docking/heading_error',
            10,
        )
        self.lateral_publisher = self.create_publisher(
            Float32,
            '/docking/lateral_error',
            10,
        )
        self.front_distance_publisher = self.create_publisher(
            Float32,
            '/docking/front_distance',
            10,
        )
        self.rear_distance_publisher = self.create_publisher(
            Float32,
            '/docking/rear_distance',
            10,
        )

        self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10,
        )
        self.create_subscription(
            String,
            command_topic,
            self.command_callback,
            10,
        )
        self.create_subscription(
            String,
            arbiter_state_topic,
            self.arbiter_state_callback,
            state_qos,
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_scan: Optional[LaserScan] = None
        self.scan_rx_time: Optional[float] = None
        self.odom_rx_time: Optional[float] = None
        self.odom_stamp: Optional[float] = None
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.arbiter_state = ''
        self.arbiter_rx_time: Optional[float] = None
        self.robot_pose = None
        self.robot_pose_time: Optional[float] = None

        self.state = 'IDLE'
        self.operation = ''
        self.active_station = ''
        self.operation_start_time: Optional[float] = None
        self.phase_start_time: Optional[float] = None
        self.stable_since: Optional[float] = None
        self.previous_heading_error = 0.0
        self.filtered_derivative = 0.0
        self.previous_control_time: Optional[float] = None
        self.last_linear_command = 0.0
        self.last_angular_command = 0.0
        self.last_command_time: Optional[float] = None
        self.last_status = ''
        self.sequence_session = uuid.uuid4().hex
        self.sequence_id = ''
        self.sequence_record = {'request_id': '', 'status': 'IDLE', 'station': ''}
        self.sequence_cache = OrderedDict()
        self.docked_station = ''
        self.manager_rx_time = -math.inf
        self.sequence_timer = self.create_timer(0.20, self.sequence_heartbeat)

        self.control_timer = self.create_timer(
            1.0 / self.control_frequency,
            self.control_loop,
        )

        self.publish_zero()
        self.publish_status('IDLE')
        self.get_logger().info(
            f'Đã nạp {len(self.stations)} trạm; nhận lệnh tại '
            f'{command_topic}; scan trước={self.front_scan_angle:.3f} rad, '
            f'scan sau={self.rear_scan_angle:.3f} rad.'
        )

    def string_parameter(self, name):
        return str(self.get_parameter(name).value)

    def positive_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} phải là số hữu hạn lớn hơn 0.')
        return value

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def manager_heartbeat(self, msg):
        self.manager_rx_time = time.monotonic()

    def sequence_heartbeat(self):
        self.sequence_publish(self.sequence_record)

    def sequence_publish(self, record):
        now = self.now_seconds()
        fresh = self.data_is_fresh(self.scan_rx_time, now)
        if self.latest_scan is not None:
            stamp = self.latest_scan.header.stamp
            age = now - stamp.sec - stamp.nanosec * 1e-9
            fresh = fresh and -0.10 <= age <= self.sensor_timeout
        front = self.sector_minimum(self.front_scan_angle) if fresh else None
        rear = self.sector_minimum(self.rear_scan_angle) if fresh else None
        data = dict(record, session_id=self.sequence_session, phase=self.state,
                    station_hash=self.station_hash, docked_station=self.docked_station,
                    front_distance=front, rear_distance=rear, scan_fresh=fresh,
                    target_front_distance=self.target_front_distance,
                    front_distance_tolerance=self.front_distance_tolerance)
        msg = String()
        msg.data = json.dumps(data, allow_nan=False)
        self.sequence_publisher.publish(msg)
        if front is not None:
            self.publish_float(self.front_distance_publisher, front)
        if rear is not None:
            self.publish_float(self.rear_distance_publisher, rear)

    def sequence_reply(self, request_id, status, station):
        record = {'request_id': request_id, 'status': status, 'station': station}
        self.sequence_cache[request_id] = record
        while len(self.sequence_cache) > 64:
            self.sequence_cache.popitem(last=False)
        self.sequence_publish(record)

    def sequence_callback(self, msg):
        try:
            data = json.loads(msg.data)
            request_id, verb, station = data['request_id'], data['verb'], data.get('station', '')
            if not isinstance(request_id, str) or not 1 <= len(request_id) <= 128:
                raise ValueError('Invalid request_id')
            if not isinstance(verb, str) or not isinstance(station, str):
                raise ValueError('Invalid command')
        except (ValueError, TypeError, KeyError, AttributeError):
            self.get_logger().warning('Bỏ qua sequence command không hợp lệ.')
            return
        if verb == 'cancel':
            if request_id == self.sequence_id and self.state in self.ACTIVE_STATES:
                self.cancel_operation()
            else:
                # Remember cancellation even if the start command arrives later.
                previous = self.sequence_cache.get(request_id, {})
                self.sequence_reply(request_id, 'CANCELED', previous.get('station', station))
            return
        if request_id in self.sequence_cache:
            self.sequence_publish(self.sequence_cache[request_id])
            return
        if verb == 'reset':
            if self.state in self.ACTIVE_STATES:
                self.sequence_reply(request_id, 'RESET_BUSY', station)
                return
            self.publish_zero()
            self.request_arbiter_mode('HOLD')
            self.sequence_id, self.state = request_id, 'IDLE'
            self.operation = self.active_station = self.last_status = ''
            self.publish_status('IDLE')
            return
        if verb not in ('dock', 'undock') or station not in self.stations:
            self.sequence_reply(request_id, 'INVALID_COMMAND', station)
            return
        if self.state in self.ACTIVE_STATES or self.state == 'FAULT':
            self.sequence_reply(request_id, 'BUSY_OR_FAULT', station)
            return
        if self.managed_only and time.monotonic() - self.manager_rx_time > 1.0:
            self.sequence_reply(request_id, 'MANAGER_UNAVAILABLE', station)
            return
        if not self.arbiter_state.startswith('HOLD:'):
            self.sequence_reply(request_id, 'ARBITER_NOT_HOLD', station)
            return
        self.sequence_id, self.last_status = request_id, ''
        self.start_operation(verb.upper(), station)

    @staticmethod
    def require_number(container, key, context):
        if key not in container:
            raise ValueError(f'Thiếu {context}.{key}')
        value = float(container[key])
        if not math.isfinite(value):
            raise ValueError(f'{context}.{key} phải là số hữu hạn.')
        return value

    def load_pose(self, raw, context):
        if not isinstance(raw, dict):
            raise ValueError(f'{context} không hợp lệ.')
        return {
            'x': self.require_number(raw, 'x', context),
            'y': self.require_number(raw, 'y', context),
            'yaw': self.require_number(raw, 'yaw', context),
        }

    def load_stations(self, station_file):
        path = Path(station_file).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f'Không tìm thấy station_file: {path}')

        with path.open('r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
        if not isinstance(data, dict):
            raise ValueError('stations.yaml phải chứa một YAML mapping.')

        system = data.get('station_system', {})
        frame_id = str(system.get('frame_id', 'map'))
        raw_stations = data.get('stations', {})
        stations = {}
        for key, raw in raw_stations.items():
            if not isinstance(raw, dict):
                continue
            if str(raw.get('type', 'station')).lower() != 'station':
                continue
            staging = self.load_pose(
                raw.get('staging_pose'),
                f'{key}.staging_pose',
            )
            dock = self.load_pose(raw.get('dock_pose'), f'{key}.dock_pose')
            path_length = math.hypot(
                dock['x'] - staging['x'],
                dock['y'] - staging['y'],
            )
            if path_length < 0.10:
                raise ValueError(f'Đường docking của {key} quá ngắn.')
            stations[str(key)] = {
                'display_name': str(raw.get('display_name', key)),
                'staging_pose': staging,
                'dock_pose': dock,
                'path_length': path_length,
            }

        if not stations:
            raise ValueError('Không có trạm docking hợp lệ trong stations.yaml.')
        return frame_id, stations

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.scan_rx_time = self.now_seconds()

    def odom_callback(self, msg):
        self.linear_velocity = float(msg.twist.twist.linear.x)
        self.angular_velocity = float(msg.twist.twist.angular.z)
        self.odom_rx_time = self.now_seconds()
        self.odom_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def arbiter_state_callback(self, msg):
        self.arbiter_state = msg.data
        self.arbiter_rx_time = self.now_seconds()

    def normalize_station(self, raw):
        normalized = raw.strip().lower().replace('-', '_').replace(' ', '_')
        aliases = {
            '1': 'station_1',
            'station1': 'station_1',
            'tram1': 'station_1',
            'tram_1': 'station_1',
            '2': 'station_2',
            'station2': 'station_2',
            'tram2': 'station_2',
            'tram_2': 'station_2',
        }
        return aliases.get(normalized, normalized)

    def parse_command(self, raw):
        cleaned = raw.strip().lower().replace(':', ' ')
        parts = cleaned.split()
        if not parts:
            return '', ''
        verb = parts[0]
        station = self.normalize_station(parts[1]) if len(parts) > 1 else ''
        return verb, station

    def command_callback(self, msg):
        verb, station = self.parse_command(msg.data)

        if self.managed_only and verb not in ('cancel', 'stop'):
            self.get_logger().warning('Chế độ AUTO: dùng /station/command; không gửi dock/undock trực tiếp.')
            return

        if verb in ('cancel', 'stop'):
            self.cancel_operation()
            return
        if verb == 'reset':
            self.reset_fault()
            return
        if verb not in ('dock', 'undock'):
            self.publish_status('INVALID_COMMAND', station)
            self.get_logger().error(
                'Lệnh hợp lệ: "dock station_1", "undock station_1", '
                '"cancel" hoặc "reset".'
            )
            return
        if station not in self.stations:
            self.publish_status('INVALID_STATION', station)
            self.get_logger().error(f'Không tồn tại trạm {station}.')
            return
        if self.state in self.ACTIVE_STATES:
            self.publish_status('BUSY', self.active_station)
            return
        if self.state == 'FAULT':
            self.publish_status('RESET_REQUIRED', station)
            return

        self.sequence_id = ''
        self.start_operation(verb.upper(), station)

    def update_robot_pose(self, now):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException:
            return False

        stamp = transform.header.stamp
        stamp_seconds = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
        if not -0.10 <= now - stamp_seconds <= self.tf_timeout:
            return False

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        self.robot_pose = (
            float(translation.x),
            float(translation.y),
            quaternion_to_yaw(rotation),
        )
        if not all(math.isfinite(value) for value in self.robot_pose):
            return False
        self.robot_pose_time = now
        return True

    def data_is_fresh(self, receipt_time, now, timeout=None):
        if timeout is None:
            timeout = self.sensor_timeout
        return receipt_time is not None and 0 <= now - receipt_time <= timeout

    def sensors_ready(self, now):
        if not self.update_robot_pose(now):
            return False, 'TF_STALE'
        if not self.data_is_fresh(self.scan_rx_time, now):
            return False, 'SCAN_STALE'
        stamp = self.latest_scan.header.stamp
        if not -0.10 <= now - stamp.sec - stamp.nanosec * 1e-9 <= self.sensor_timeout:
            return False, 'SCAN_STAMP_INVALID'
        if not self.data_is_fresh(self.odom_rx_time, now):
            return False, 'ODOM_STALE'
        if self.odom_stamp is None or not -0.10 <= now - self.odom_stamp <= self.sensor_timeout:
            return False, 'ODOM_STAMP_INVALID'
        if not all(math.isfinite(v) for v in (self.linear_velocity, self.angular_velocity)):
            return False, 'INVALID_ODOMETRY'
        if not self.data_is_fresh(
            self.arbiter_rx_time,
            now,
            self.arbiter_timeout,
        ):
            return False, 'ARBITER_UNAVAILABLE'
        return True, ''

    def start_operation(self, operation, station_key):
        now = self.now_seconds()
        ready, reason = self.sensors_ready(now)
        if not ready:
            self.publish_status(reason, station_key)
            self.get_logger().error(f'Không thể bắt đầu: {reason}.')
            return

        station = self.stations[station_key]
        if operation == 'DOCK':
            expected = station['staging_pose']
            tolerance = self.start_position_tolerance
        else:
            expected = station['dock_pose']
            tolerance = self.undock_start_position_tolerance

        distance = math.hypot(
            self.robot_pose[0] - expected['x'],
            self.robot_pose[1] - expected['y'],
        )
        if distance > tolerance:
            reason = 'NOT_AT_STAGING' if operation == 'DOCK' else 'NOT_DOCKED'
            self.publish_status(reason, station_key)
            self.get_logger().error(
                f'{reason}: sai số vị trí={distance:.3f} m, '
                f'giới hạn={tolerance:.3f} m.'
            )
            return

        self.operation = operation
        self.active_station = station_key
        self.operation_start_time = now
        self.phase_start_time = now
        self.stable_since = None
        self.previous_control_time = None
        self.previous_heading_error = 0.0
        self.filtered_derivative = 0.0
        self.state = 'WAIT_STOP'
        self.request_arbiter_mode('HOLD')
        self.publish_zero()
        self.publish_status(f'{operation}_WAIT_STOP', station_key)
        self.get_logger().info(
            f'Bắt đầu {operation} tại {station_key}; sai số điểm đầu '
            f'{distance:.3f} m.'
        )

    def request_arbiter_mode(self, mode):
        msg = String()
        msg.data = mode
        self.mode_publisher.publish(msg)

    def publish_status(self, state, station_key=''):
        status = f'{state}:{station_key}' if station_key else state
        if status == self.last_status:
            return
        msg = String()
        msg.data = status
        self.status_publisher.publish(msg)
        self.last_status = status
        self.sequence_record = {'request_id': self.sequence_id, 'status': state, 'station': station_key}
        if self.sequence_id:
            self.sequence_reply(self.sequence_id, state, station_key)
        else:
            self.sequence_publish(self.sequence_record)

    def publish_zero(self):
        self.velocity_publisher.publish(Twist())
        self.last_linear_command = 0.0
        self.last_angular_command = 0.0
        self.last_command_time = self.now_seconds()

    def publish_motion(self, linear, angular, now):
        dt = 1.0 / self.control_frequency
        if self.last_command_time is not None:
            dt = clamp(now - self.last_command_time, 1.0e-3, 0.20)

        linear_step = self.linear_acceleration_limit * dt
        angular_step = self.angular_acceleration_limit * dt
        limited_linear = clamp(
            linear,
            self.last_linear_command - linear_step,
            self.last_linear_command + linear_step,
        )
        limited_angular = clamp(
            angular,
            self.last_angular_command - angular_step,
            self.last_angular_command + angular_step,
        )

        command = Twist()
        command.linear.x = limited_linear
        command.angular.z = limited_angular
        self.velocity_publisher.publish(command)
        self.last_linear_command = limited_linear
        self.last_angular_command = limited_angular
        self.last_command_time = now

    def publish_float(self, publisher, value):
        msg = Float32()
        msg.data = float(value)
        publisher.publish(msg)

    def cancel_operation(self):
        self.publish_zero()
        self.request_arbiter_mode('HOLD')
        station = self.active_station
        self.state = 'IDLE'
        self.operation = ''
        self.active_station = ''
        self.publish_status('CANCELED', station)

    def reset_fault(self):
        if self.state != 'FAULT':
            self.publish_status('IDLE')
            return
        self.publish_zero()
        self.request_arbiter_mode('HOLD')
        self.state = 'IDLE'
        self.operation = ''
        self.active_station = ''
        self.publish_status('IDLE')

    def set_fault(self, reason):
        station = self.active_station
        self.publish_zero()
        self.request_arbiter_mode('FAULT')
        self.state = 'FAULT'
        self.publish_status(f'FAULT_{reason}', station)
        self.get_logger().error(f'Docking FAULT: {reason}')

    def sector_minimum(self, center_angle):
        scan = self.latest_scan
        if scan is None or scan.angle_increment == 0.0:
            return None

        minimum = math.inf
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            if abs(normalize_angle(angle - center_angle)) <= (
                self.scan_sector_half_width
            ):
                minimum = min(minimum, float(distance))

        return minimum if math.isfinite(minimum) else None

    def stopped_is_stable(self, now):
        stopped = (
            abs(self.linear_velocity) <= self.stop_linear_threshold
            and abs(self.angular_velocity) <= self.stop_angular_threshold
        )
        if not stopped:
            self.stable_since = None
            return False
        if self.stable_since is None:
            self.stable_since = now
        return (now - self.stable_since) >= self.stop_stable_time

    def enter_state(self, state, now):
        self.state = state
        self.phase_start_time = now
        self.stable_since = None
        self.previous_control_time = None
        self.previous_heading_error = 0.0
        self.filtered_derivative = 0.0
        self.publish_status(
            f'{self.operation}_{state}',
            self.active_station,
        )

    def angular_command(self, error, gain):
        command = clamp(
            gain * error,
            -self.maximum_angular_speed,
            self.maximum_angular_speed,
        )
        if abs(error) > self.align_yaw_tolerance:
            command_sign = 1.0 if command >= 0.0 else -1.0
            command = command_sign * max(
                abs(command),
                self.minimum_angular_speed,
            )
        return command

    def align_to_yaw(self, target_yaw, now):
        if now - self.phase_start_time > self.alignment_timeout:
            self.set_fault('ALIGNMENT_TIMEOUT')
            return False

        error = normalize_angle(target_yaw - self.robot_pose[2])
        self.publish_float(self.heading_publisher, error)

        if abs(error) <= self.align_yaw_tolerance:
            self.publish_zero()

            # Phải đạt yaw và vận tốc thực tế đã xuống dưới
            # ngưỡng dừng liên tục đủ stop_stable_time.
            return self.stopped_is_stable(now)

        self.stable_since = None

        angular = self.angular_command(error, self.yaw_kp)
        angular = clamp(
            angular,
            -self.alignment_max_angular_speed,
            self.alignment_max_angular_speed,
        )

        # Chỉ xoay tại chỗ trong giai đoạn căn hướng.
        self.publish_motion(0.0, angular, now)
        return False

    def line_geometry(self):
        station = self.stations[self.active_station]
        if self.operation == 'DOCK':
            start = station['staging_pose']
            end = station['dock_pose']
            reverse = False
        else:
            start = station['dock_pose']
            end = station['staging_pose']
            reverse = True
        return start, end, reverse

    def follow_line(self, now, front_distance, rear_distance):
        start, end, reverse = self.line_geometry()
        dx = end['x'] - start['x']
        dy = end['y'] - start['y']
        length = math.hypot(dx, dy)
        ux = dx / length
        uy = dy / length

        rx = self.robot_pose[0] - start['x']
        ry = self.robot_pose[1] - start['y']
        projection = clamp(rx * ux + ry * uy, 0.0, length)
        lateral_error = -uy * rx + ux * ry
        remaining = math.hypot(
            end['x'] - self.robot_pose[0],
            end['y'] - self.robot_pose[1],
        )

        self.publish_float(self.distance_publisher, remaining)
        self.publish_float(self.lateral_publisher, lateral_error)

        if abs(lateral_error) > self.path_lateral_limit:
            self.set_fault('LATERAL_ERROR')
            return

        if reverse:
            if rear_distance is None:
                self.set_fault('NO_REAR_SCAN')
                return
            if rear_distance <= self.rear_obstacle_stop_distance:
                self.set_fault('REAR_OBSTACLE')
                return
        else:
            if front_distance is None:
                self.set_fault('NO_FRONT_SCAN')
                return
            if front_distance <= self.front_hard_stop_distance:
                self.set_fault('FRONT_TOO_CLOSE')
                return
            distance_reached = (
                front_distance
                <= self.target_front_distance + self.front_distance_tolerance
            )
            if distance_reached and remaining <= self.front_distance_assist_window:
                self.publish_zero()
                self.enter_state('ALIGN_FINAL', now)
                return
            if distance_reached and remaining > self.front_distance_assist_window:
                self.set_fault('UNEXPECTED_FRONT_OBSTACLE')
                return

        if remaining <= self.path_position_tolerance:
            self.publish_zero()
            self.enter_state('ALIGN_FINAL', now)
            return

        lookahead_s = min(length, projection + self.lookahead_distance)
        lookahead_x = start['x'] + lookahead_s * ux
        lookahead_y = start['y'] + lookahead_s * uy
        motion_yaw = math.atan2(
            lookahead_y - self.robot_pose[1],
            lookahead_x - self.robot_pose[0],
        )
        desired_robot_yaw = normalize_angle(
            motion_yaw + (math.pi if reverse else 0.0)
        )
        heading_error = normalize_angle(
            desired_robot_yaw - self.robot_pose[2]
        )
        self.publish_float(self.heading_publisher, heading_error)

        dt = 1.0 / self.control_frequency
        if self.previous_control_time is not None:
            dt = max(1.0e-3, now - self.previous_control_time)
        raw_derivative = normalize_angle(
            heading_error - self.previous_heading_error
        ) / dt
        alpha = self.derivative_filter_alpha
        self.filtered_derivative = (
            alpha * raw_derivative
            + (1.0 - alpha) * self.filtered_derivative
        )

        angular = (
            self.heading_kp * heading_error
            + self.heading_kd * self.filtered_derivative
        )
        angular = clamp(
            angular,
            -self.maximum_angular_speed,
            self.maximum_angular_speed,
        )

        maximum_speed = self.reverse_speed if reverse else self.forward_speed
        distance_scale = clamp(
            remaining / self.slowdown_distance,
            self.minimum_speed / maximum_speed,
            1.0,
        )
        heading_scale = clamp(
            1.0 - abs(heading_error) / self.drive_heading_limit,
            0.0,
            1.0,
        )
        linear = maximum_speed * distance_scale * heading_scale
        if heading_scale > 0.0:
            linear = max(linear, self.minimum_speed)
        else:
            linear = 0.0
        if reverse:
            linear = -linear

        self.publish_motion(linear, angular, now)

        self.previous_heading_error = heading_error
        self.previous_control_time = now

    def verify_final(self, now, front_distance):
        station = self.stations[self.active_station]
        target = (
            station['dock_pose']
            if self.operation == 'DOCK'
            else station['staging_pose']
        )
        position_error = math.hypot(
            target['x'] - self.robot_pose[0],
            target['y'] - self.robot_pose[1],
        )
        yaw_error = abs(normalize_angle(target['yaw'] - self.robot_pose[2]))

        position_ok = position_error <= self.final_position_tolerance
        yaw_ok = yaw_error <= self.final_yaw_tolerance
        distance_ok = True
        if self.operation == 'DOCK':
            distance_ok = (
                front_distance is not None
                and abs(front_distance - self.target_front_distance)
                <= self.front_distance_tolerance
            )

        self.publish_float(self.distance_publisher, position_error)
        self.publish_float(self.heading_publisher, yaw_error)

        stopped = (abs(self.linear_velocity) <= self.stop_linear_threshold
                   and abs(self.angular_velocity) <= self.stop_angular_threshold)
        if position_ok and yaw_ok and distance_ok and stopped:
            if self.stable_since is None:
                self.stable_since = now
            if (now - self.stable_since) >= self.final_stable_time:
                self.complete_operation()
            return

        self.stable_since = None
        if (
            self.phase_start_time is not None
            and (now - self.phase_start_time) > self.final_condition_timeout
        ):
            self.set_fault('FINAL_CONDITION_MISMATCH')

    def complete_operation(self):
        station = self.active_station
        operation = self.operation
        self.publish_zero()

        if operation == 'DOCK':
            self.request_arbiter_mode('HOLD')
            self.state = 'DOCKED'
            self.docked_station = station
            self.publish_status('DOCKED', station)
        else:
            # AUTO manager must verify departure before handing control to Nav2.
            self.request_arbiter_mode('HOLD' if self.sequence_id else 'NAV2')
            self.state = 'UNDOCKED'
            self.docked_station = ''
            self.publish_status('UNDOCKED', station)

        self.operation = ''
        self.active_station = ''
        self.get_logger().info(f'{operation} hoàn tất tại {station}.')

    def control_loop(self):
        now = self.now_seconds()
        if self.state not in self.ACTIVE_STATES:
            self.publish_zero()
            return
        if self.managed_only and time.monotonic() - self.manager_rx_time > 1.0:
            self.set_fault('MANAGER_LOST')
            return

        if (
            self.operation_start_time is not None
            and (now - self.operation_start_time) > self.operation_timeout
        ):
            self.set_fault('TIMEOUT')
            return

        ready, reason = self.sensors_ready(now)
        if not ready:
            self.set_fault(reason)
            return

        front_distance = self.sector_minimum(self.front_scan_angle)
        rear_distance = self.sector_minimum(self.rear_scan_angle)
        if front_distance is not None:
            self.publish_float(self.front_distance_publisher, front_distance)
        if rear_distance is not None:
            self.publish_float(self.rear_distance_publisher, rear_distance)

        station = self.stations[self.active_station]

        if self.state == 'WAIT_STOP':
            self.publish_zero()
            distance = front_distance if self.operation == 'DOCK' else rear_distance
            threshold = (self.front_hard_stop_distance if self.operation == 'DOCK'
                         else self.rear_obstacle_stop_distance)
            if distance is None or distance <= threshold:
                self.set_fault('START_SCAN_MISSING_OR_OBSTACLE')
                return
            if self.stopped_is_stable(now):
                self.request_arbiter_mode('DOCKING')
                self.enter_state('ALIGN_START', now)
            return

        if not self.arbiter_state.startswith('DOCKING:'):
            if self.phase_start_time is not None and (
                now - self.phase_start_time
            ) > self.arbiter_timeout:
                self.set_fault('ARBITER_NOT_DOCKING')
            else:
                self.publish_zero()
            return

        if self.state == 'ALIGN_START':
            target_yaw = station['dock_pose']['yaw']

            start_pose = station[
                'staging_pose' if self.operation == 'DOCK' else 'dock_pose'
            ]
            position_limit = (
                self.start_position_tolerance
                if self.operation == 'DOCK'
                else self.undock_start_position_tolerance
            )

            position_error = math.hypot(
                self.robot_pose[0] - start_pose['x'],
                self.robot_pose[1] - start_pose['y'],
            )

            # Không tiếp tục xoay nếu xe đã ra khỏi vùng cho phép.
            if position_error > position_limit:
                self.set_fault('ALIGN_START_POSITION_OUT_OF_RANGE')
                return

            yaw_error = normalize_angle(
                target_yaw - self.robot_pose[2]
            )

            if (
                self.operation == 'DOCK'
                and abs(yaw_error) > self.start_yaw_tolerance
            ):
                self.set_fault('ALIGN_START_YAW_OUT_OF_RANGE')
                return

            # Giữ kiểm tra khoảng cách trong suốt bước căn đầu.
            distance = (
                front_distance
                if self.operation == 'DOCK'
                else rear_distance
            )
            threshold = (
                self.front_hard_stop_distance
                if self.operation == 'DOCK'
                else self.rear_obstacle_stop_distance
            )

            if distance is None or distance <= threshold:
                self.set_fault('ALIGN_SCAN_MISSING_OR_OBSTACLE')
                return

            if self.align_to_yaw(target_yaw, now):
                self.enter_state('FOLLOW_PATH', now)
            return

        if self.state == 'FOLLOW_PATH':
            self.follow_line(now, front_distance, rear_distance)
            return

        if self.state == 'ALIGN_FINAL':
            target_yaw = (
                station['dock_pose']['yaw']
                if self.operation == 'DOCK'
                else station['staging_pose']['yaw']
            )
            if self.align_to_yaw(target_yaw, now):
                self.enter_state('VERIFY_FINAL', now)
            return

        if self.state == 'VERIFY_FINAL':
            self.publish_zero()
            self.verify_final(now, front_distance)

    def emergency_stop(self):
        self.publish_zero()
        self.request_arbiter_mode('FAULT')


def main(args=None):
    rclpy.init(args=args)
    node = DockingControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.emergency_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
