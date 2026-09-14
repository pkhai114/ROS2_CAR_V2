"""Exercise production node methods with fake ROS messages/actions (no ROS runtime).

AST loading omits ROS imports and constructors only; tested methods are compiled
unchanged from the delivered source files, not reimplemented by these tests.
"""

import ast
import hashlib
import json
import math
import sys
import unittest
import uuid
from collections import OrderedDict
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as NS
from typing import Optional
from unittest.mock import Mock

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
from test_station_workflow import STATIONS, StationWorkflow, departure_station, pose_error


class Twist:
    def __init__(self):
        self.linear = NS(x=0., y=0., z=0.)
        self.angular = NS(x=0., y=0., z=0.)


def load_class(relative, name):
    source = ast.parse((ROOT / relative).read_text())
    module = ast.Module(body=[n for n in source.body if isinstance(n, ast.ClassDef)
                              or isinstance(n, ast.FunctionDef) and n.name != 'main'],
                        type_ignores=[])
    symbols = dict(Node=object, math=math, json=json, uuid=uuid, Path=Path,
                   OrderedDict=OrderedDict, Optional=Optional, hashlib=hashlib,
                   time=NS(monotonic=lambda: 100.), Twist=Twist, String=NS, Float32=NS,
                   StationWorkflow=StationWorkflow, departure_station=departure_station,
                   pose_error=pose_error, GoalStatus=NS(STATUS_SUCCEEDED=4),
                   TransformException=ValueError, yaml=yaml)
    exec(compile(module, str(ROOT / relative), 'exec'), symbols)
    return symbols[name]


Manager = load_class('robot_navigation_bringup/scripts/station_auto_manager_node.py', 'StationAutoManager')
Controller = load_class('robot_docking/scripts/docking_controller_node.py', 'DockingControllerNode')
Arbiter = load_class('robot_docking/scripts/velocity_arbiter_node.py', 'VelocityArbiterNode')


def manager():
    m = Manager.__new__(Manager)
    m.p = yaml.safe_load((ROOT / 'robot_navigation_bringup/config/station_auto_params.yaml').read_text())['station_manager_node']['ros__parameters']
    m.stations, m.flow = STATIONS, StationWorkflow(STATIONS)
    m.session_id, m.station_hash = 'manager-session', 'stations-sha'
    for attr in ('event_pub', 'status_pub', 'current_pub', 'target_pub', 'mode_pub',
                 'dock_pub', 'heartbeat_pub', 'distance_pub'):
        setattr(m, attr, Mock())
    m.get_logger = Mock(return_value=Mock())
    m.get_clock = Mock(return_value=NS(now=lambda: NS(nanoseconds=100_000_000_000)))
    m.arbiter_state, m.arbiter_rx = 'HOLD:ZERO', 100.
    m.mode_requested, m.mode_request_time = 'HOLD', 99.
    m.odom_rx, m.odom_stamp, m.dock_rx = 100., 100., 100.
    m.vx = m.vy = m.wz = 0.
    m.dock_session = 'controller-session'
    m.dock_report = dict(session_id=m.dock_session, phase='IDLE', station_hash='stations-sha',
                         scan_fresh=True, front_distance=.37,
                         target_front_distance=.37, front_distance_tolerance=.04)
    m.nav_context = None
    m.nav_active_ids, m.owned_nav_ids = set(), set()
    m.reset_pending = m.dock_cancel_pending = m.last_flow_state = m.last_status = ''
    m.stable_since = None
    m.reset_deadline = math.inf
    m.robot_pose = Mock(return_value=(0., 0., 0.))
    m.get_publishers_info_by_topic = Mock(return_value=[NS(node_name='velocity_arbiter')])
    m.count_publishers = Mock(return_value=1)
    m.count_subscribers = Mock(return_value=1)
    return m


def controller():
    d = Controller.__new__(Controller)
    d.get_logger = Mock(return_value=Mock())
    d.now_seconds = lambda: 100.
    d.sequence_session, d.sequence_id, d.station_hash = 'controller-session', '', 'stations-sha'
    d.sequence_record = {'request_id': '', 'status': 'IDLE', 'station': ''}
    d.sequence_cache = OrderedDict()
    d.state, d.operation, d.active_station, d.docked_station = 'IDLE', '', '', ''
    d.last_status = ''
    d.managed_only, d.manager_rx_time = True, 100.
    d.scan_rx_time = d.odom_rx_time = d.odom_stamp = d.arbiter_rx_time = 100.
    d.latest_scan = NS(header=NS(stamp=NS(sec=100, nanosec=0)))
    d.sensor_timeout, d.arbiter_timeout = .5, 1.
    d.front_scan_angle, d.rear_scan_angle = math.pi / 2, -math.pi / 2
    d.target_front_distance, d.front_distance_tolerance = .37, .04
    d.front_hard_stop_distance, d.rear_obstacle_stop_distance = .33, .37
    d.sector_minimum = lambda angle: .37 if angle > 0 else 2.
    d.arbiter_state = 'HOLD:ZERO'
    d.stations = {k: {'staging_pose': v['target_pose'], 'dock_pose': v['dock_pose']}
                  for k, v in STATIONS.items() if v['type'] == 'station'}
    p = STATIONS['station_1']['target_pose']
    d.robot_pose = (p['x'], p['y'], p['yaw'])
    d.start_position_tolerance, d.undock_start_position_tolerance = .18, .14
    d.linear_velocity = d.angular_velocity = 0.
    d.stop_linear_threshold, d.stop_angular_threshold = .015, .035
    d.sensors_ready = Mock(return_value=(True, ''))
    d.publish_zero, d.request_arbiter_mode = Mock(), Mock()
    for attr in ('sequence_publisher', 'status_publisher', 'front_distance_publisher',
                 'rear_distance_publisher', 'distance_publisher', 'heading_publisher'):
        setattr(d, attr, Mock())
    return d


def sequence(node, verb, request_id='request-1', station='station_1'):
    node.sequence_callback(NS(data=json.dumps(dict(verb=verb, request_id=request_id, station=station))))


def arbiter():
    a = Arbiter.__new__(Arbiter)
    a.require_station_heartbeat, a.station_heartbeat_timeout = True, 1.
    a.station_rx, a.station_session = 100., 'manager-session'
    a.now_seconds = lambda: 100.
    a.mode, a.transition_until = 'NAV2', 99.
    a.source_timeout, a.switch_hold_time = .3, .15
    a.nav_command, a.dock_command = Twist(), Twist()
    a.nav_command.linear.x = .1
    a.nav_rx_time, a.dock_rx_time = 100., None
    a.output_publisher, a.state_publisher = Mock(), Mock()
    a.count_publishers = Mock(return_value=1)
    a.get_logger = Mock(return_value=Mock())
    return a


class ManagerTests(unittest.TestCase):
    def test_staging_position_yaw_velocity_gates(self):
        m = manager()
        m.flow.state, m.flow.target = 'CHECK_STAGING', 'station_1'
        p = STATIONS['station_1']['target_pose']
        pose = (p['x'], p['y'], p['yaw'])
        self.assertTrue(m.gate(pose))
        self.assertFalse(m.gate((pose[0] + .11, pose[1], pose[2])))
        self.assertTrue(m.gate((pose[0], pose[1], pose[2] + math.radians(8.7))))
        self.assertFalse(m.gate((pose[0], pose[1], pose[2] + math.radians(16))))
        m.vx = .02
        self.assertFalse(m.gate(pose))
        m.vx, m.wz = 0., .04
        self.assertFalse(m.gate(pose))

    def test_docked_requires_valid_front_distance(self):
        m = manager()
        m.flow.state, m.flow.target = 'CHECK_DOCK', 'station_1'
        p = STATIONS['station_1']['dock_pose']
        pose = (p['x'], p['y'], p['yaw'])
        self.assertTrue(m.gate(pose))
        for value in (None, float('nan'), float('inf'), .5):
            m.dock_report['front_distance'] = value
            self.assertFalse(m.gate(pose))

    def test_undock_gate_uses_departing_station(self):
        m = manager()
        m.flow.state, m.flow.target, m.flow.departing = 'CHECK_UNDOCK', 'station_2', 'station_1'
        p = STATIONS['station_1']['target_pose']
        self.assertTrue(m.gate((p['x'], p['y'], p['yaw'])))
        p = STATIONS['station_2']['target_pose']
        self.assertFalse(m.gate((p['x'], p['y'], p['yaw'])))

    def test_fresh_health_and_rejection_cases(self):
        self.assertEqual(manager().health(100.), '')
        cases = [('arbiter_rx', 98., 'ARBITER_LOST'), ('dock_rx', 98., 'DOCKING_LOST'),
                 ('odom_rx', 98., 'ODOM_LOST'), ('odom_stamp', 98., 'ODOM_STAMP_INVALID'),
                 ('vx', float('nan'), 'INVALID_ODOMETRY')]
        for attr, value, reason in cases:
            m = manager()
            setattr(m, attr, value)
            self.assertEqual(m.health(100.), reason)
        m = manager()
        m.dock_report['scan_fresh'] = False
        self.assertEqual(m.health(100.), 'SCAN_LOST')
        m = manager()
        m.station_hash = 'another-file'
        self.assertEqual(m.health(100.), 'STATION_FILE_MISMATCH')

    def test_mode_ack_must_not_accept_wait_manager(self):
        m = manager()
        self.assertTrue(m.mode_ack('HOLD'))
        for state in ('HOLD:TRANSITION', 'HOLD:WAIT_MANAGER', 'FAULT:ZERO'):
            m.arbiter_state = state
            self.assertFalse(m.mode_ack('HOLD'))
        m.arbiter_state, m.arbiter_rx = 'HOLD:ZERO', 98.
        self.assertFalse(m.mode_ack('HOLD'))

    def test_duplicate_velocity_publishers_block_start(self):
        m = manager()
        m.get_publishers_info_by_topic.return_value.append(NS(node_name='velocity_smoother'))
        m.command(NS(data='station_1'))
        self.assertEqual(m.flow.state, 'IDLE')
        self.assertIn('NOT_READY', m.event_pub.publish.call_args.args[0].data)

    def test_station_command_routes_to_workflow(self):
        m = manager()
        m.command(NS(data='station_1'))
        self.assertEqual(m.flow.state, 'HOLD_BEFORE_NAV')
        self.assertEqual(m.mode_pub.publish.call_args.args[0].data, 'HOLD')

    def test_command_at_dock_starts_undock(self):
        m = manager()
        p = STATIONS['station_1']['dock_pose']
        m.robot_pose.return_value = (p['x'], p['y'], p['yaw'])
        m.command(NS(data='station_2'))
        self.assertEqual(m.flow.state, 'HOLD_BEFORE_UNDOCK')

    def test_wrong_heading_at_dock_prevents_large_rotation(self):
        m = manager()
        p = STATIONS['station_1']['dock_pose']
        m.robot_pose.return_value = (p['x'], p['y'], p['yaw'] + math.pi)
        m.command(NS(data='station_2'))
        self.assertEqual(m.flow.state, 'IDLE')

    def test_correlated_peer_restart_aborts(self):
        m = manager()
        m.flow.start('station_1', '', 99.)
        m.docking_callback(NS(data=json.dumps(dict(m.dock_report, session_id='new-session'))))
        self.assertEqual(m.flow.state, 'CANCELING')
        self.assertEqual(m.flow.failure_reason, 'DOCKING_RESTARTED')

    def pending_nav(self):
        m = manager()
        m.flow.start('station_1', '', 90.)
        m.flow.hold_ready(91.)
        m.flow.nav_mode_ready(92.)
        m.flow.drain()
        context = dict(token=m.flow.nav_token, handle=None, cancel=False, cancel_sent=False)
        m.nav_context = context
        result = Future()
        handle = NS(accepted=True, goal_id=NS(uuid=list(range(16))),
                    get_result_async=lambda: result, cancel_goal_async=Mock())
        accepted = Future()
        accepted.set_result(handle)
        return m, context, handle, accepted, result

    def test_cancel_before_accept_cancels_late_handle(self):
        m, ctx, handle, accepted, result = self.pending_nav()
        m.flow.cancel(99.)
        m.apply_effects()
        m.nav_accepted(accepted, ctx)
        handle.cancel_goal_async.assert_called_once()
        self.assertEqual(m.flow.state, 'CANCELING')
        result.set_result(NS(status=4))
        self.assertIsNone(m.nav_context)
        self.assertEqual(m.flow.state, 'CANCELING')
        m.dock_pub.publish.assert_not_called()

    def test_immediate_result_after_accept_not_lost(self):
        m, ctx, handle, accepted, result = self.pending_nav()
        result.set_result(NS(status=4))
        m.nav_accepted(accepted, ctx)
        self.assertEqual(m.flow.state, 'CHECK_STAGING')
        self.assertIsNone(m.nav_context)

    def test_busy_station_command_does_not_replace_target(self):
        m = manager()
        m.command(NS(data='station_1'))
        m.command(NS(data='station_2'))
        self.assertEqual(m.flow.target, 'station_1')

    def test_cancel_requires_ack_and_stable_stop(self):
        m = manager()
        m.flow.cancel(99.)
        m.apply_effects()
        m.tick()
        self.assertEqual(m.flow.state, 'CANCELING')
        m.stable_since = 99.
        m.tick()
        self.assertEqual(m.flow.state, 'CANCELED')

    def test_reset_waits_for_controller_reply(self):
        m = manager()
        m.flow.state = 'FAILED'
        m.command(NS(data='reset'))
        self.assertEqual(m.flow.state, 'FAILED')
        request = json.loads(m.dock_pub.publish.call_args.args[0].data)
        m.docking_callback(NS(data=json.dumps(dict(m.dock_report, request_id=request['request_id'], status='IDLE'))))
        self.assertEqual(m.flow.state, 'IDLE')

    def test_staging_requires_stable_interval_before_command(self):
        m = manager()
        m.flow.target, m.flow.state = 'station_1', 'CHECK_STAGING'
        p = STATIONS['station_1']['target_pose']
        m.robot_pose.return_value = (p['x'], p['y'], p['yaw'])
        m.tick()
        m.dock_pub.publish.assert_not_called()
        m.stable_since = 99.4
        m.tick()
        self.assertEqual(m.flow.state, 'WAIT_DOCK_ACK')
        self.assertEqual(json.loads(m.dock_pub.publish.call_args.args[0].data)['verb'], 'dock')

    def test_lost_scan_aborts_active_navigation(self):
        m = manager()
        m.flow.state, m.flow.target = 'NAVIGATING', 'station_1'
        m.arbiter_state = 'NAV2:ACTIVE'
        m.dock_report['scan_fresh'] = False
        m.tick()
        self.assertEqual(m.flow.state, 'CANCELING')
        self.assertEqual(m.flow.failure_reason, 'SCAN_LOST')


class ProtocolTests(unittest.TestCase):
    def test_stale_scan_stamp_rejected_despite_recent_receipt(self):
        d = controller()
        d.update_robot_pose = Mock(return_value=True)
        d.latest_scan.header.stamp.sec = 98
        self.assertEqual(Controller.sensors_ready(d, 100.), (False, 'SCAN_STAMP_INVALID'))

    def test_stale_odom_stamp_rejected_despite_recent_receipt(self):
        d = controller()
        d.update_robot_pose = Mock(return_value=True)
        d.odom_stamp = 98.
        self.assertEqual(Controller.sensors_ready(d, 100.), (False, 'ODOM_STAMP_INVALID'))

    def test_final_check_requires_stopped_odometry(self):
        d = controller()
        d.operation, d.active_station = 'DOCK', 'station_1'
        p = STATIONS['station_1']['dock_pose']
        d.robot_pose = (p['x'], p['y'], p['yaw'])
        d.final_position_tolerance, d.final_yaw_tolerance = .08, math.radians(5.)
        d.final_stable_time, d.final_condition_timeout = .5, 3.
        d.phase_start_time, d.stable_since = 100., None
        d.complete_operation = Mock()
        d.linear_velocity = .02
        d.verify_final(100., .37)
        d.complete_operation.assert_not_called()
        self.assertIsNone(d.stable_since)
        d.linear_velocity = 0.
        d.verify_final(100.1, .37)
        d.verify_final(100.7, .37)
        d.complete_operation.assert_called_once()

    def test_rear_obstacle_prevents_undock_rotation_start(self):
        d = controller()
        d.operation, d.active_station, d.state = 'UNDOCK', 'station_1', 'WAIT_STOP'
        d.operation_start_time, d.operation_timeout = 99., 30.
        d.sector_minimum = lambda _: .20
        d.control_loop()
        self.assertEqual(d.state, 'FAULT')
        self.assertNotIn(unittest.mock.call('DOCKING'), d.request_arbiter_mode.call_args_list)

    def test_managed_command_ack_and_duplicate_not_restarted(self):
        d = controller()
        sequence(d, 'dock')
        self.assertEqual(d.state, 'WAIT_STOP')
        report = json.loads(d.sequence_publisher.publish.call_args.args[0].data)
        self.assertEqual((report['request_id'], report['status']), ('request-1', 'DOCK_WAIT_STOP'))
        d.start_operation = Mock()
        sequence(d, 'dock')
        d.start_operation.assert_not_called()

    def test_cancel_arrives_before_start_tombstone(self):
        d = controller()
        d.start_operation = Mock()
        sequence(d, 'cancel')
        sequence(d, 'dock')
        d.start_operation.assert_not_called()
        self.assertEqual(d.sequence_cache['request-1']['status'], 'CANCELED')

    def test_cancel_active_sequence(self):
        d = controller()
        sequence(d, 'dock')
        sequence(d, 'cancel')
        self.assertEqual(d.state, 'IDLE')
        self.assertEqual(d.sequence_cache['request-1']['status'], 'CANCELED')
        d.request_arbiter_mode.assert_called_with('HOLD')

    def test_managed_undock_retains_hold(self):
        d = controller()
        d.sequence_id, d.operation, d.active_station = 'seq', 'UNDOCK', 'station_1'
        d.docked_station = 'station_1'
        d.complete_operation()
        d.request_arbiter_mode.assert_called_with('HOLD')
        self.assertEqual(d.state, 'UNDOCKED')
        self.assertEqual(d.docked_station, '')

    def test_manual_undock_legacy_mode(self):
        d = controller()
        d.operation, d.active_station = 'UNDOCK', 'station_1'
        d.complete_operation()
        d.request_arbiter_mode.assert_called_with('NAV2')

    def test_manual_dock_disabled_in_auto(self):
        d = controller()
        d.start_operation = Mock()
        d.command_callback(NS(data='dock station_1'))
        d.start_operation.assert_not_called()

    def test_bad_json_does_not_move(self):
        d = controller()
        d.start_operation = Mock()
        for raw in ('', '{}', '[]', 'null', '{bad}', '{"request_id": 7, "verb": "dock"}'):
            d.sequence_callback(NS(data=raw))
        d.start_operation.assert_not_called()

    def test_no_manager_no_start(self):
        d = controller()
        d.manager_rx_time = 98.
        sequence(d, 'dock')
        self.assertEqual(d.state, 'IDLE')
        self.assertEqual(d.sequence_cache['request-1']['status'], 'MANAGER_UNAVAILABLE')

    def test_active_manager_loss_faults(self):
        d = controller()
        d.state, d.active_station, d.manager_rx_time = 'FOLLOW_PATH', 'station_1', 98.
        d.control_loop()
        self.assertEqual(d.state, 'FAULT')
        d.request_arbiter_mode.assert_called_with('FAULT')

    def test_reset_refuses_active_operation(self):
        d = controller()
        sequence(d, 'dock')
        sequence(d, 'reset', 'reset-1')
        self.assertEqual(d.state, 'WAIT_STOP')
        self.assertEqual(d.sequence_cache['reset-1']['status'], 'RESET_BUSY')

    def test_dock_completion_records_current_station(self):
        d = controller()
        d.operation, d.active_station = 'DOCK', 'station_1'
        d.complete_operation()
        report = json.loads(d.sequence_publisher.publish.call_args.args[0].data)
        self.assertEqual((report['status'], report['docked_station']), ('DOCKED', 'station_1'))

    def test_actual_manager_controller_protocol(self):
        m, d = manager(), controller()
        m.flow.target = 'station_1'
        m.flow.begin_docking('dock', 'station_1', 99.)
        m.apply_effects()
        d.sequence_callback(m.dock_pub.publish.call_args.args[0])
        m.docking_callback(d.sequence_publisher.publish.call_args.args[0])
        self.assertEqual(m.flow.state, 'DOCKING')
        d.complete_operation()
        m.docking_callback(d.sequence_publisher.publish.call_args.args[0])
        self.assertEqual(m.flow.state, 'CHECK_DOCK')
        self.assertNotEqual(m.flow.state, 'DOCKED')


class ArbiterTests(unittest.TestCase):
    def test_only_selected_source_passes(self):
        a = arbiter()
        a.control_timer()
        self.assertEqual(a.output_publisher.publish.call_args.args[0].linear.x, .1)

    def test_manager_loss_zero_fault(self):
        a = arbiter()
        a.station_rx = 98.
        a.control_timer()
        self.assertEqual(a.mode, 'FAULT')
        self.assertEqual(a.output_publisher.publish.call_args.args[0].linear.x, 0.)

    def test_duplicate_manager_stops(self):
        a = arbiter()
        a.count_publishers.return_value = 2
        a.control_timer()
        self.assertEqual(a.mode, 'FAULT')

    def test_manager_restart_faults(self):
        a = arbiter()
        a.station_callback(NS(data='new-session'))
        self.assertEqual(a.mode, 'FAULT')

    def test_stale_velocity_zero(self):
        a = arbiter()
        a.nav_rx_time = 99.
        a.control_timer()
        self.assertEqual(a.output_publisher.publish.call_args.args[0].linear.x, 0.)
        self.assertEqual(a.state_publisher.publish.call_args.args[0].data, 'NAV2:STALE')

    def test_mode_switch_discards_old_velocities(self):
        a = arbiter()
        a.dock_rx_time = 100.
        a.mode_callback(NS(data='DOCKING'))
        self.assertIsNone(a.dock_rx_time)
        self.assertIsNone(a.nav_rx_time)
        a.control_timer()
        self.assertEqual(a.output_publisher.publish.call_args.args[0].linear.x, 0.)

    def test_invalid_velocity_faults(self):
        a = arbiter()
        bad = Twist()
        bad.linear.x = float('nan')
        a.dock_callback(bad)
        self.assertEqual(a.mode, 'FAULT')

    def test_fault_cannot_directly_reenable_motion(self):
        a = arbiter()
        a.mode = 'FAULT'
        a.mode_callback(NS(data='NAV2'))
        self.assertEqual(a.mode, 'FAULT')
        a.mode_callback(NS(data='HOLD'))
        self.assertEqual(a.mode, 'HOLD')


if __name__ == '__main__':
    unittest.main()
