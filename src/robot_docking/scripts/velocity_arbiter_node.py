#!/usr/bin/env python3

"""Select exactly one velocity source before commands reach the ESP32."""

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class VelocityArbiterNode(Node):
    """Fail-safe selector for Nav2 and docking velocity commands."""

    VALID_MODES = ('NAV2', 'DOCKING', 'HOLD', 'FAULT')

    def __init__(self):
        super().__init__('velocity_arbiter')

        self.declare_parameter('nav_input_topic', '/cmd_vel_nav_output')
        self.declare_parameter('dock_input_topic', '/cmd_vel_dock')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('mode_command_topic', '/velocity_arbiter/mode')
        self.declare_parameter('mode_state_topic', '/velocity_arbiter/state')
        self.declare_parameter('publish_frequency', 20.0)
        self.declare_parameter('source_timeout', 0.30)
        self.declare_parameter('switch_hold_time', 0.15)
        self.declare_parameter('default_mode', 'NAV2')
        self.declare_parameter('require_station_heartbeat', False)
        self.declare_parameter('station_heartbeat_timeout', 1.0)
        self.require_station_heartbeat = bool(self.get_parameter('require_station_heartbeat').value)
        self.station_heartbeat_timeout = float(self.get_parameter('station_heartbeat_timeout').value)
        if not math.isfinite(self.station_heartbeat_timeout) or self.station_heartbeat_timeout <= 0:
            raise ValueError('station_heartbeat_timeout phải là số dương hữu hạn.')
        self.station_rx = None
        self.station_session = ''
        self.create_subscription(String, '/station/heartbeat', self.station_callback, 10)

        nav_topic = str(self.get_parameter('nav_input_topic').value)
        dock_topic = str(self.get_parameter('dock_input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        mode_topic = str(self.get_parameter('mode_command_topic').value)
        state_topic = str(self.get_parameter('mode_state_topic').value)

        frequency = float(self.get_parameter('publish_frequency').value)
        self.source_timeout = float(
            self.get_parameter('source_timeout').value
        )
        self.switch_hold_time = float(
            self.get_parameter('switch_hold_time').value
        )

        if not math.isfinite(frequency) or frequency <= 0.0:
            raise ValueError('publish_frequency phải lớn hơn 0.')
        if not math.isfinite(self.source_timeout) or self.source_timeout <= 0.0:
            raise ValueError('source_timeout phải lớn hơn 0.')
        if not math.isfinite(self.switch_hold_time) or self.switch_hold_time < 0.0:
            raise ValueError('switch_hold_time không được âm.')

        default_mode = str(
            self.get_parameter('default_mode').value
        ).strip().upper()
        if default_mode not in self.VALID_MODES:
            raise ValueError(
                f'default_mode phải thuộc {self.VALID_MODES}.'
            )

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.output_publisher = self.create_publisher(
            Twist,
            output_topic,
            10,
        )
        self.state_publisher = self.create_publisher(
            String,
            state_topic,
            state_qos,
        )

        self.create_subscription(Twist, nav_topic, self.nav_callback, 10)
        self.create_subscription(Twist, dock_topic, self.dock_callback, 10)
        self.create_subscription(String, mode_topic, self.mode_callback, 10)

        self.nav_command = Twist()
        self.dock_command = Twist()
        self.nav_rx_time: Optional[float] = None
        self.dock_rx_time: Optional[float] = None
        self.mode = default_mode
        self.transition_until = self.now_seconds() + self.switch_hold_time
        self.last_state = ''

        self.timer = self.create_timer(1.0 / frequency, self.control_timer)

        self.publish_zero()
        self.publish_state(f'{self.mode}:TRANSITION')
        self.get_logger().info(
            f'Arbiter sẵn sàng: {nav_topic} hoặc {dock_topic} -> '
            f'{output_topic}; mode mặc định={self.mode}'
        )

    def now_seconds(self):
        return time.monotonic()

    def station_callback(self, msg):
        if self.require_station_heartbeat and self.station_session and msg.data != self.station_session:
            self.mode = 'FAULT'
            self.publish_zero()
        self.station_session = msg.data
        self.station_rx = self.now_seconds()

    @staticmethod
    def valid_command(msg):
        return all(math.isfinite(v) for v in (msg.linear.x, msg.linear.y, msg.linear.z,
                                              msg.angular.x, msg.angular.y, msg.angular.z))

    def nav_callback(self, msg):
        if not self.valid_command(msg):
            self.emergency_stop()
            return
        self.nav_command = msg
        self.nav_rx_time = self.now_seconds()

    def dock_callback(self, msg):
        if not self.valid_command(msg):
            self.emergency_stop()
            return
        self.dock_command = msg
        self.dock_rx_time = self.now_seconds()

    def normalize_mode(self, raw_mode):
        normalized = raw_mode.strip().upper().replace('-', '_')
        aliases = {
            'DOCK': 'DOCKING',
            'STOP': 'HOLD',
            'ESTOP': 'FAULT',
            'E_STOP': 'FAULT',
        }
        return aliases.get(normalized, normalized)

    def mode_callback(self, msg):
        requested = self.normalize_mode(msg.data)
        if requested not in self.VALID_MODES:
            self.get_logger().error(
                f'Bỏ qua mode "{msg.data}"; hợp lệ: {self.VALID_MODES}'
            )
            return

        if requested == self.mode:
            return
        if self.mode == 'FAULT' and requested not in ('HOLD', 'FAULT'):
            self.get_logger().warning('FAULT: yêu cầu HOLD/reset trước khi cấp quyền chuyển động.')
            return

        previous = self.mode
        self.mode = requested
        self.transition_until = self.now_seconds() + self.switch_hold_time

        # Một lệnh mới phải tới sau lần đổi mode; không tái sử dụng lệnh cũ.
        self.nav_rx_time = None
        self.dock_rx_time = None
        self.publish_zero()
        self.publish_state(f'{self.mode}:TRANSITION')
        self.get_logger().info(f'Đổi mode {previous} -> {self.mode}')

    def publish_state(self, state):
        # Phát định kỳ để docking_controller biết arbiter còn hoạt động.
        msg = String()
        msg.data = state
        self.state_publisher.publish(msg)
        self.last_state = state

    def publish_zero(self):
        self.output_publisher.publish(Twist())

    def source_is_fresh(self, rx_time, now):
        return rx_time is not None and (now - rx_time) <= self.source_timeout

    def control_timer(self):
        now = self.now_seconds()

        if self.require_station_heartbeat:
            fresh = self.station_rx is not None and now - self.station_rx <= self.station_heartbeat_timeout
            if not fresh or self.count_publishers('/station/heartbeat') != 1:
                if self.station_rx is not None:
                    self.mode = 'FAULT'
                self.publish_zero()
                self.publish_state(f'{self.mode}:WAIT_MANAGER')
                return

        if now < self.transition_until:
            self.publish_zero()
            self.publish_state(f'{self.mode}:TRANSITION')
            return

        if self.mode in ('HOLD', 'FAULT'):
            self.publish_zero()
            self.publish_state(f'{self.mode}:ZERO')
            return

        if self.mode == 'NAV2':
            command = self.nav_command
            fresh = self.source_is_fresh(self.nav_rx_time, now)
        else:
            command = self.dock_command
            fresh = self.source_is_fresh(self.dock_rx_time, now)

        if not fresh:
            self.publish_zero()
            self.publish_state(f'{self.mode}:STALE')
            return

        self.output_publisher.publish(command)
        self.publish_state(f'{self.mode}:ACTIVE')

    def emergency_stop(self):
        self.mode = 'FAULT'
        self.publish_zero()


def main(args=None):
    rclpy.init(args=args)
    node = VelocityArbiterNode()

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
