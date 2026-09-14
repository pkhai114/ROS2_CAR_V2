#!/usr/bin/env python3
"""Coordinate undock -> Nav2 staging -> dock through acknowledged handoffs."""

import hashlib
import json
import math
import time
import uuid
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformException, TransformListener

from station_workflow import StationWorkflow, departure_station, pose_error


class StationAutoManager(Node):
    def __init__(self):
        super().__init__('station_manager_node')
        defaults = {
            'station_file': '', 'command_topic': '/station/command',
            'status_topic': '/station/status', 'current_topic': '/station/current',
            'distance_remaining_topic': '/station/distance_remaining',
            'navigate_action': '/navigate_to_pose', 'base_frame': 'base_link',
            'odom_topic': '/odometry/filtered',
            'staging_position_tolerance': 0.10, 'staging_yaw_tolerance_deg': 5.0,
            'staging_alignment_max_yaw_deg': 15.0,
            'dock_position_tolerance': 0.08, 'dock_yaw_tolerance_deg': 5.0,
            'stop_linear_threshold': 0.015, 'stop_angular_threshold': 0.035,
            'stable_time': 0.50, 'sensor_timeout': 0.50, 'tf_timeout': 0.50,
            'peer_timeout': 1.0, 'handoff_timeout': 5.0,
            'settle_timeout': 8.0, 'navigation_timeout': 180.0,
            'docking_timeout': 35.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        for key, value in self.p.items():
            if isinstance(defaults[key], float) and (
                    not math.isfinite(value) or value <= 0):
                raise ValueError(f'{key} phải là số dương hữu hạn.')
        raw_bytes = Path(self.p['station_file']).expanduser().read_bytes()
        self.station_hash = hashlib.sha256(raw_bytes).hexdigest()
        data = yaml.safe_load(raw_bytes)
        self.frame = str(data.get('station_system', {}).get('frame_id', 'map'))
        self.stations = {}
        for key, raw in data['stations'].items():
            kind = raw.get('type', 'station')
            if kind not in ('home', 'station'):
                raise ValueError(f'Loại trạm không hợp lệ: {key}')
            target = self.read_pose(raw['pose' if kind == 'home' else 'staging_pose'])
            entry = {'type': kind, 'target_pose': target}
            if kind == 'station':
                entry['dock_pose'] = self.read_pose(raw['dock_pose'])
                if pose_error((target['x'], target['y'], target['yaw']),
                              entry['dock_pose'])[0] < 0.10:
                    raise ValueError(f'Đường docking quá ngắn: {key}')
            self.stations[str(key)] = entry

        self.flow = StationWorkflow(self.stations, self.p['handoff_timeout'],
                                    self.p['settle_timeout'],
                                    self.p['navigation_timeout'],
                                    self.p['docking_timeout'])
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, self.p['status_topic'], retained)
        self.current_pub = self.create_publisher(String, self.p['current_topic'], retained)
        self.target_pub = self.create_publisher(String, '/station/target', retained)
        self.event_pub = self.create_publisher(String, '/station/events', 10)
        self.distance_pub = self.create_publisher(Float32, self.p['distance_remaining_topic'], 10)
        self.mode_pub = self.create_publisher(String, '/velocity_arbiter/mode', 10)
        self.dock_pub = self.create_publisher(String, '/docking/sequence_command', 10)
        self.heartbeat_pub = self.create_publisher(String, '/station/heartbeat', 10)
        self.create_subscription(String, self.p['command_topic'], self.command, 10)
        self.create_subscription(String, '/velocity_arbiter/state', self.arbiter_callback, retained)
        self.create_subscription(String, '/docking/sequence_status', self.docking_callback, 10)
        self.create_subscription(Odometry, self.p['odom_topic'], self.odom_callback, 10)
        self.create_subscription(GoalStatusArray, self.p['navigate_action'] + '/_action/status',
                                 self.nav_status_callback, retained)
        self.nav = ActionClient(self, NavigateToPose, self.p['navigate_action'])
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.session_id = uuid.uuid4().hex
        self.arbiter_state, self.arbiter_rx = '', -math.inf
        self.dock_report, self.dock_rx, self.dock_session = {}, -math.inf, ''
        self.odom_rx, self.odom_stamp = -math.inf, -math.inf
        self.vx = self.vy = self.wz = math.inf
        self.mode_requested, self.mode_request_time = '', -math.inf
        self.nav_context = None
        self.nav_active_ids, self.owned_nav_ids = set(), set()
        self.dock_cancel_pending = ''
        self.reset_pending = ''
        self.reset_deadline = math.inf
        self.stable_since = None
        self.last_flow_state = ''
        self.last_status = ''
        self.timer = self.create_timer(0.10, self.tick)
        self.publish_status('IDLE', '')
        self.send(self.current_pub, '')
        self.get_logger().info('Station AUTO: station_1 / station_2 / home / cancel / reset')

    @staticmethod
    def read_pose(raw):
        result = {key: float(raw[key]) for key in ('x', 'y', 'yaw')}
        if not all(math.isfinite(v) for v in result.values()):
            raise ValueError('Pose phải là số hữu hạn.')
        return result

    @staticmethod
    def send(publisher, value):
        msg = String()
        msg.data = value
        publisher.publish(msg)

    def event(self, text):
        self.send(self.event_pub, text)
        self.get_logger().warning(text)

    def publish_status(self, state, station):
        text = f'{state}:{station}' if station else state
        if text != self.last_status:
            self.send(self.status_pub, text)
            self.get_logger().info(text)
            self.last_status = text

    def mode(self, value):
        self.mode_requested, self.mode_request_time = value, time.monotonic()
        self.send(self.mode_pub, value)

    def mode_ack(self, value):
        allowed = {'HOLD': {'HOLD:ZERO'},
                   'NAV2': {'NAV2:ACTIVE', 'NAV2:STALE'}}
        return (self.arbiter_rx >= self.mode_request_time
                and time.monotonic() - self.arbiter_rx <= self.p['peer_timeout']
                and self.arbiter_state in allowed.get(value, set()))

    def arbiter_callback(self, msg):
        self.arbiter_state, self.arbiter_rx = msg.data, time.monotonic()

    def odom_callback(self, msg):
        self.odom_rx = time.monotonic()
        self.odom_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.vx, self.vy, self.wz = (msg.twist.twist.linear.x,
                                    msg.twist.twist.linear.y, msg.twist.twist.angular.z)

    def nav_status_callback(self, msg):
        self.nav_active_ids = {bytes(s.goal_info.goal_id.uuid).hex() for s in msg.status_list
                               if s.status in (1, 2, 3)}

    def docking_callback(self, msg):
        try:
            report = json.loads(msg.data)
            if not isinstance(report, dict) or not report.get('session_id'):
                return
        except (ValueError, TypeError):
            return
        now = time.monotonic()
        new_session = report['session_id']
        if self.dock_session and new_session != self.dock_session and self.flow.busy:
            self.flow.fail('DOCKING_RESTARTED', now)
        self.dock_session, self.dock_report, self.dock_rx = new_session, report, now
        if (report.get('request_id') == self.dock_cancel_pending
                and report.get('phase') not in ('WAIT_STOP', 'ALIGN_START', 'FOLLOW_PATH',
                                                 'ALIGN_FINAL', 'VERIFY_FINAL')):
            self.dock_cancel_pending = ''
        if self.reset_pending and report.get('request_id') == self.reset_pending:
            if report.get('status') == 'IDLE' and report.get('phase') == 'IDLE':
                self.reset_pending = ''
                self.flow.reset(now)
            elif report.get('status') == 'RESET_BUSY':
                self.reset_pending = ''
                self.event('RESET_REFUSED: docking còn hoạt động')
        self.flow.docking_report(report, now)
        self.apply_effects()

    def robot_pose(self):
        transform = self.tf.lookup_transform(self.frame, self.p['base_frame'], Time())
        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        if not -0.10 <= ros_now - stamp <= self.p['tf_timeout']:
            raise ValueError('TF_STALE_OR_FUTURE')
        q = transform.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        pose = (transform.transform.translation.x, transform.transform.translation.y, yaw)
        if not all(math.isfinite(v) for v in pose):
            raise ValueError('INVALID_TF')
        return pose

    def health(self, now):
        if now - self.arbiter_rx > self.p['peer_timeout']:
            return 'ARBITER_LOST'
        if now - self.dock_rx > self.p['peer_timeout']:
            return 'DOCKING_LOST'
        if self.dock_report.get('station_hash') != self.station_hash:
            return 'STATION_FILE_MISMATCH'
        if now - self.odom_rx > self.p['sensor_timeout']:
            return 'ODOM_LOST'
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        if not -0.10 <= ros_now - self.odom_stamp <= self.p['sensor_timeout']:
            return 'ODOM_STAMP_INVALID'
        if not all(math.isfinite(v) for v in (self.vx, self.vy, self.wz)):
            return 'INVALID_ODOMETRY'
        if not self.dock_report.get('scan_fresh', False):
            return 'SCAN_LOST'
        return ''

    def graph_ready(self):
        publishers = self.get_publishers_info_by_topic('/cmd_vel')
        return (len(publishers) == 1 and publishers[0].node_name == 'velocity_arbiter'
                and self.count_publishers('/station/heartbeat') == 1
                and self.count_publishers('/docking/sequence_status') == 1
                and self.count_subscribers('/docking/sequence_command') == 1
                and self.count_subscribers(self.p['command_topic']) == 1)

    def command(self, msg):
        value = msg.data.strip().lower().replace('-', '_').replace(' ', '_')
        aliases = {'1': 'station_1', 'station1': 'station_1', 'tram1': 'station_1',
                   '2': 'station_2', 'station2': 'station_2', 'tram2': 'station_2', '0': 'home'}
        target = aliases.get(value, value)
        now = time.monotonic()
        if target in ('cancel', 'stop'):
            self.flow.cancel(now)
            self.apply_effects()
            return
        if target == 'reset':
            if self.flow.busy or self.nav_context is not None or self.dock_cancel_pending:
                self.event('RESET_REFUSED: chưa xác nhận hủy xong tất cả thao tác')
                return
            if now - self.dock_rx > self.p['peer_timeout']:
                self.event('RESET_REFUSED: không có heartbeat docking')
                return
            self.reset_pending = uuid.uuid4().hex
            self.reset_deadline = now + self.p['handoff_timeout']
            self.mode('HOLD')
            self.send(self.dock_pub, json.dumps({'request_id': self.reset_pending,
                                                'verb': 'reset', 'station': ''}))
            return
        if target not in self.stations:
            self.event('INVALID_COMMAND: ' + value)
            return
        if self.reset_pending or self.flow.busy:
            self.event('BUSY: ' + self.flow.target + '; không xếp hàng lệnh mới')
            return
        if self.flow.state in ('FAILED', 'CANCELED'):
            self.event('RESET_REQUIRED: ' + self.flow.failure_reason)
            return
        reason = self.health(now)
        if reason or not self.graph_ready():
            self.event('NOT_READY: ' + (reason or 'VELOCITY_OR_NODE_GRAPH_INVALID'))
            return
        if self.arbiter_state.startswith('FAULT:'):
            self.event('RESET_REQUIRED: arbiter đang FAULT')
            return
        if self.nav_active_ids - self.owned_nav_ids or self.nav_context is not None:
            self.event('NAV2_BUSY: hủy goal ngoài station manager trước')
            return
        if self.dock_report.get('phase') not in ('IDLE', 'DOCKED', 'UNDOCKED'):
            self.event('DOCKING_BUSY_OR_FAULT')
            return
        try:
            pose = self.robot_pose()
            departure = departure_station(pose, self.stations)
            if departure and pose_error(pose, self.stations[departure]['dock_pose'])[1] > math.radians(
                    self.p['dock_yaw_tolerance_deg']):
                raise ValueError('DOCK_HEADING_MISMATCH: không tự xoay lớn sát trạm')
            remembered = self.flow.docked_station or self.dock_report.get('docked_station', '')
            if remembered and departure != remembered:
                raise ValueError('DOCK_STATE_POSE_MISMATCH: kiểm tra vị trí xe')
        except (TransformException, ValueError) as error:
            self.event('NOT_READY: ' + str(error))
            return
        self.dock_session = self.dock_report['session_id']
        result = self.flow.start(target, departure, now)
        self.send(self.target_pub, target)
        self.event(result + ': ' + target)
        self.apply_effects()

    def execute_navigation(self, station, token):
        if self.nav_context is not None or not self.nav.server_is_ready():
            self.flow.fail('NAV2_UNAVAILABLE_OR_BUSY', time.monotonic())
            return
        pose = self.stations[station]['target_pose']
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = pose['x'], pose['y']
        goal.pose.pose.orientation.z = math.sin(pose['yaw'] / 2)
        goal.pose.pose.orientation.w = math.cos(pose['yaw'] / 2)
        context = {'token': token, 'handle': None, 'cancel': False, 'cancel_sent': False}
        self.nav_context = context
        try:
            future = self.nav.send_goal_async(goal, feedback_callback=self.feedback)
            future.add_done_callback(lambda f: self.nav_accepted(f, context))
        except Exception as error:
            self.nav_context = None
            self.flow.fail('NAV2_SEND_ERROR:' + str(error), time.monotonic())

    def nav_accepted(self, future, context):
        try:
            handle = future.result()
        except Exception as error:
            if self.nav_context is context:
                self.nav_context = None
            self.flow.fail('NAV2_ACCEPT_ERROR:' + str(error), time.monotonic())
            self.apply_effects()
            return
        if not handle.accepted:
            if self.nav_context is context:
                self.nav_context = None
            self.flow.nav_accepted(context['token'], False, time.monotonic())
        else:
            context['handle'] = handle
            self.owned_nav_ids.add(bytes(handle.goal_id.uuid).hex())
            if (context['cancel'] or context['token'] != self.flow.nav_token
                    or self.flow.state != 'SENDING_GOAL'):
                self.cancel_nav(context)
            else:
                self.flow.nav_accepted(context['token'], True, time.monotonic())
            try:
                handle.get_result_async().add_done_callback(lambda f: self.nav_result(f, context))
            except Exception as error:
                self.flow.fail('NAV2_RESULT_REQUEST_ERROR:' + str(error), time.monotonic())
        self.apply_effects()

    def nav_result(self, future, context):
        try:
            succeeded = future.result().status == GoalStatus.STATUS_SUCCEEDED
        except Exception:
            succeeded = False
        if self.nav_context is context:
            self.nav_context = None
        self.flow.nav_result(context['token'], succeeded, time.monotonic())
        self.apply_effects()

    def feedback(self, message):
        if self.flow.state == 'NAVIGATING':
            out = Float32()
            out.data = float(message.feedback.distance_remaining)
            self.distance_pub.publish(out)

    def cancel_nav(self, context):
        if context is None:
            return
        context['cancel'] = True
        if context['handle'] is not None and not context['cancel_sent']:
            context['cancel_sent'] = True
            try:
                context['handle'].cancel_goal_async()
            except Exception as error:
                self.event('NAV2_CANCEL_ERROR: ' + str(error))

    def apply_effects(self):
        while self.flow.effects:
            for kind, data in self.flow.drain():
                if kind == 'mode':
                    self.mode(data['mode'])
                elif kind == 'status':
                    station = data['station']
                    if self.flow.departing and data['state'] in (
                            'HOLD_BEFORE_UNDOCK', 'WAIT_UNDOCK_ACK', 'UNDOCKING', 'CHECK_UNDOCK'):
                        station = self.flow.departing
                    self.publish_status(data['state'], station)
                    if data['state'] in ('DOCKED', 'AT_HOME'):
                        self.send(self.current_pub, station)
                    elif data['state'] in ('HOLD_BEFORE_NAV', 'HOLD_BEFORE_UNDOCK',
                                          'CANCELING', 'IDLE'):
                        self.send(self.current_pub, '')
                    if data['state'] in ('FAILED', 'CANCELED'):
                        self.event(self.flow.failure_reason)
                elif kind == 'navigate':
                    self.execute_navigation(data['station'], data['token'])
                elif kind == 'docking':
                    self.send(self.dock_pub, json.dumps({'request_id': data['request_id'],
                              'verb': data['operation'], 'station': data['station']}))
                elif kind == 'cancel_children':
                    self.cancel_nav(self.nav_context)
                    if data['request_id']:
                        self.dock_cancel_pending = data['request_id']
                        self.send(self.dock_pub, json.dumps({'request_id': data['request_id'],
                                                           'verb': 'cancel', 'station': ''}))

    def gate(self, pose):
        state = self.flow.state
        key = self.flow.departing if state == 'CHECK_UNDOCK' else self.flow.target
        dock = state == 'CHECK_DOCK'
        target = self.stations[key]['dock_pose' if dock else 'target_pose']
        distance, angle = pose_error(pose, target)
        out = Float32()
        out.data = distance
        self.distance_pub.publish(out)
        position_limit = self.p['dock_position_tolerance' if dock else 'staging_position_tolerance']
        yaw_limit = math.radians(self.p['dock_yaw_tolerance_deg' if dock else 'staging_yaw_tolerance_deg'])
        # Chỉ nới góc bàn giao khi tới staging của một trạm.
        # HOME, kiểm tra sau undock và kiểm tra dock cuối giữ ngưỡng cũ.
        if state == 'CHECK_STAGING' and self.stations[key]['type'] == 'station':
            _, angle = pose_error(pose, self.stations[key]['dock_pose'])
            yaw_limit = math.radians(self.p['staging_alignment_max_yaw_deg'])
        valid = (distance <= position_limit and angle <= yaw_limit
                 and math.hypot(self.vx, self.vy) <= self.p['stop_linear_threshold']
                 and abs(self.wz) <= self.p['stop_angular_threshold'])
        if dock:
            front = self.dock_report.get('front_distance')
            expected = self.dock_report.get('target_front_distance')
            tolerance = self.dock_report.get('front_distance_tolerance')
            valid = (valid and all(isinstance(v, (int, float)) and math.isfinite(v)
                                  for v in (front, expected, tolerance))
                     and abs(front - expected) <= tolerance)
        return valid

    def tick(self):
        now = time.monotonic()
        self.send(self.heartbeat_pub, self.session_id)
        if self.reset_pending and now > self.reset_deadline:
            self.reset_pending = ''
            self.event('RESET_TIMEOUT')
        if not self.flow.busy:
            return
        if self.last_flow_state != self.flow.state:
            self.stable_since = None
            self.last_flow_state = self.flow.state
        if self.flow.state == 'CANCELING':
            if (self.nav_context is None and not self.dock_cancel_pending
                    and self.mode_ack('HOLD')
                    and math.hypot(self.vx, self.vy) <= self.p['stop_linear_threshold']
                    and abs(self.wz) <= self.p['stop_angular_threshold']
                    and now - self.odom_rx <= self.p['sensor_timeout']):
                if self.stable_since is None:
                    self.stable_since = now
                if now - self.stable_since >= self.p['stable_time']:
                    self.flow.children_stopped(now)
            else:
                self.stable_since = None
            self.flow.tick(now)
            self.apply_effects()
            return
        reason = self.health(now)
        if not reason and self.arbiter_state.startswith('FAULT:'):
            reason = 'ARBITER_FAULT'
        if (not reason and self.flow.state in ('SENDING_GOAL', 'NAVIGATING')
                and not self.arbiter_state.startswith('NAV2:')):
            reason = 'NAV2_CONTROL_REVOKED'
        if not reason and not self.graph_ready():
            reason = 'VELOCITY_OR_NODE_GRAPH_CHANGED'
        # Acceptance callback may arrive after the first action status message.
        pending_accept = self.nav_context and self.nav_context['handle'] is None
        if not reason and not pending_accept and self.nav_active_ids - self.owned_nav_ids:
            reason = 'EXTERNAL_NAV2_GOAL'
        try:
            pose = self.robot_pose()
        except (TransformException, ValueError):
            pose, reason = None, reason or 'TF_UNAVAILABLE'
        if reason:
            self.flow.fail(reason, now)
        elif self.flow.state in ('HOLD_BEFORE_UNDOCK', 'HOLD_BEFORE_NAV'):
            if self.mode_ack('HOLD'):
                self.flow.hold_ready(now)
        elif self.flow.state == 'WAIT_NAV_MODE':
            if self.mode_ack('NAV2'):
                self.flow.nav_mode_ready(now)
        elif self.flow.state in ('CHECK_STAGING', 'CHECK_UNDOCK', 'CHECK_DOCK'):
            if self.mode_ack('HOLD') and self.gate(pose):
                if self.stable_since is None:
                    self.stable_since = now
                if now - self.stable_since >= self.p['stable_time']:
                    self.flow.settled(now)
            else:
                self.stable_since = None
        self.flow.tick(now)
        self.apply_effects()

    def stop(self):
        self.flow.cancel(time.monotonic())
        self.apply_effects()


def main(args=None):
    rclpy.init(args=args)
    node = StationAutoManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
