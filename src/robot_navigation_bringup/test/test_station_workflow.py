"""Offline tests: run with unittest or pytest; no robot/ROS installation needed."""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from station_workflow import StationWorkflow, departure_station, pose_error


STATIONS = {
    'station_1': {'type': 'station',
                  'target_pose': {'x': 2.774, 'y': -0.576, 'yaw': -0.019},
                  'dock_pose': {'x': 3.174, 'y': -0.585, 'yaw': -0.019}},
    'station_2': {'type': 'station',
                  'target_pose': {'x': 0.962, 'y': -0.126, 'yaw': 1.617},
                  'dock_pose': {'x': 0.939, 'y': 0.274, 'yaw': 1.617}},
    'home': {'type': 'home', 'target_pose': {'x': 0., 'y': 0., 'yaw': 0.}},
}


class WorkflowTests(unittest.TestCase):
    def setUp(self):
        self.f = StationWorkflow(STATIONS)

    def navigation(self, station='station_1'):
        self.f.start(station, '', 0.)
        self.f.hold_ready(1.)
        self.f.nav_mode_ready(2.)
        self.f.nav_accepted(self.f.nav_token, True, 3.)

    def staging(self):
        self.navigation()
        self.f.nav_result(self.f.nav_token, True, 4.)
        self.assertEqual(self.f.state, 'CHECK_STAGING')

    def report(self, status, station=None, request_id=None):
        self.f.docking_report({'request_id': request_id or self.f.request_id,
                              'status': status, 'station': station or self.f.target}, 7.)

    def docked(self):
        self.staging()
        self.f.settled(5.)
        self.report('DOCK_WAIT_STOP')
        self.report('DOCKED')
        self.assertEqual(self.f.state, 'CHECK_DOCK')
        self.f.settled(8.)

    def test_nav_success_is_not_enough_to_start_docking(self):
        self.staging()
        self.assertFalse(any(kind == 'docking' for kind, _ in self.f.drain()))
        self.f.settled(5.)
        self.assertEqual(self.f.state, 'WAIT_DOCK_ACK')
        self.assertEqual(self.f.drain()[-1][1]['operation'], 'dock')

    def test_full_station_1(self):
        self.docked()
        self.assertEqual(self.f.state, 'DOCKED')
        self.assertEqual(self.f.docked_station, 'station_1')

    def test_transfer_requires_verified_undock(self):
        self.docked()
        self.f.drain()
        self.f.start('station_2', 'station_1', 10.)
        self.assertEqual(self.f.state, 'HOLD_BEFORE_UNDOCK')
        self.f.hold_ready(11.)
        effect = self.f.drain()[-1]
        self.assertEqual(effect[1]['operation'], 'undock')
        self.assertEqual(effect[1]['station'], 'station_1')
        self.report('UNDOCK_WAIT_STOP', 'station_1')
        self.report('UNDOCKED', 'station_1')
        self.assertEqual(self.f.state, 'CHECK_UNDOCK')
        self.assertFalse(any(k == 'navigate' for k, _ in self.f.drain()))
        self.f.settled(13.)
        self.assertEqual(self.f.state, 'HOLD_BEFORE_NAV')
        self.f.hold_ready(14.)
        self.f.nav_mode_ready(15.)
        self.assertEqual(self.f.drain()[-1][1]['station'], 'station_2')
        self.f.nav_accepted(self.f.nav_token, True, 16.)
        self.f.nav_result(self.f.nav_token, True, 17.)
        self.f.settled(18.)
        self.report('DOCK_WAIT_STOP', 'station_2')
        self.report('DOCKED', 'station_2')
        self.f.settled(20.)
        self.assertEqual((self.f.state, self.f.docked_station), ('DOCKED', 'station_2'))

    def test_home_needs_no_docking(self):
        self.navigation('home')
        self.f.nav_result(self.f.nav_token, True, 4.)
        self.f.settled(5.)
        self.assertEqual(self.f.state, 'AT_HOME')
        self.assertFalse(any(k == 'docking' for k, _ in self.f.drain()))

    def test_same_dock_is_reverified_without_motion(self):
        self.docked()
        self.f.drain()
        self.assertEqual(self.f.start('station_1', 'station_1', 9.), 'VERIFY_ALREADY_DOCKED')
        self.assertEqual(self.f.state, 'CHECK_DOCK')
        self.assertFalse(any(k in ('navigate', 'docking') for k, _ in self.f.drain()))

    def test_busy_rejects_and_does_not_queue(self):
        self.navigation()
        self.f.drain()
        self.assertEqual(self.f.start('station_2', '', 4.), 'BUSY')
        self.assertEqual(self.f.start('station_1', '', 4.), 'ALREADY_RUNNING')
        self.assertEqual(self.f.target, 'station_1')
        self.assertEqual(self.f.drain(), [])

    def test_invalid_station(self):
        self.assertEqual(self.f.start('station_3', '', 0.), 'INVALID_STATION')
        self.assertEqual(self.f.state, 'IDLE')

    def test_nav_rejection_holds_and_cancels(self):
        self.f.start('station_1', '', 0.)
        self.f.hold_ready(1.)
        self.f.nav_mode_ready(2.)
        self.f.nav_accepted(self.f.nav_token, False, 3.)
        self.assertEqual(self.f.state, 'CANCELING')
        self.assertEqual(self.f.failure_reason, 'NAV2_REJECTED')

    def test_nav_failure_does_not_dock(self):
        self.navigation()
        self.f.drain()
        self.f.nav_result(self.f.nav_token, False, 4.)
        self.assertEqual(self.f.state, 'CANCELING')
        self.assertFalse(any(k == 'docking' for k, _ in self.f.drain()))

    def test_stale_nav_results_ignored(self):
        self.navigation()
        self.f.nav_result('old', True, 4.)
        self.assertEqual(self.f.state, 'NAVIGATING')

    def test_dock_results_correlated(self):
        self.staging()
        self.f.settled(5.)
        self.report('DOCKED', request_id='old-dock')
        self.assertEqual(self.f.state, 'WAIT_DOCK_ACK')

    def test_terminal_without_start_ack_is_failure(self):
        self.staging()
        self.f.settled(5.)
        self.report('DOCKED')
        self.assertEqual(self.f.failure_reason, 'DOCKING_RESULT_WITHOUT_ACK')

    def test_wrong_station_result_is_failure(self):
        self.staging()
        self.f.settled(5.)
        self.report('DOCK_WAIT_STOP', 'station_2')
        self.assertEqual(self.f.failure_reason, 'DOCKING_STATION_MISMATCH')

    def test_dock_fault_is_failure(self):
        self.staging()
        self.f.settled(5.)
        self.report('DOCK_WAIT_STOP')
        self.report('FAULT_NO_FRONT_SCAN')
        self.assertEqual(self.f.state, 'CANCELING')

    def test_repeated_ack_does_not_extend_timeout(self):
        self.staging()
        self.f.settled(5.)
        self.report('DOCK_WAIT_STOP')
        deadline = self.f.deadline
        self.f.docking_report({'request_id': self.f.request_id,
                              'station': 'station_1', 'status': 'DOCK_FOLLOW_PATH'}, 12.)
        self.assertEqual(self.f.deadline, deadline)

    def test_cancel_late_nav_success_cannot_start_dock(self):
        self.navigation()
        self.f.cancel(4.)
        self.f.drain()
        self.f.nav_result(self.f.nav_token, True, 5.)
        self.assertEqual(self.f.state, 'CANCELING')
        self.assertEqual(self.f.drain(), [])
        self.f.children_stopped(6.)
        self.assertEqual(self.f.state, 'CANCELED')
        self.assertEqual(self.f.start('station_2', '', 7.), 'RESET_REQUIRED')

    def test_cancel_late_dock_success_ignored(self):
        self.staging()
        self.f.settled(5.)
        self.report('DOCK_WAIT_STOP')
        self.f.cancel(8.)
        self.report('DOCKED')
        self.assertEqual(self.f.state, 'CANCELING')

    def test_every_timed_phase_fails_closed(self):
        for state in ('HOLD_BEFORE_NAV', 'WAIT_NAV_MODE', 'SENDING_GOAL',
                      'NAVIGATING', 'CHECK_STAGING', 'WAIT_DOCK_ACK', 'DOCKING',
                      'CHECK_DOCK', 'HOLD_BEFORE_UNDOCK', 'WAIT_UNDOCK_ACK',
                      'UNDOCKING', 'CHECK_UNDOCK'):
            with self.subTest(state=state):
                f = StationWorkflow(STATIONS)
                f.transition(state, 0., 1.)
                f.tick(2.)
                self.assertEqual(f.state, 'CANCELING')
                f.tick(8.)
                self.assertEqual(f.state, 'FAILED')
                self.assertEqual(f.drain()[-1], ('mode', {'mode': 'FAULT'}))

    def test_reset_cannot_skip_cancellation(self):
        self.navigation()
        self.f.cancel(4.)
        self.assertFalse(self.f.reset(5.))
        self.f.children_stopped(6.)
        self.assertTrue(self.f.reset(7.))
        self.assertEqual(self.f.start('station_2', '', 8.), 'ACCEPTED')

    def test_departure_at_each_dock(self):
        for key in ('station_1', 'station_2'):
            p = STATIONS[key]['dock_pose']
            self.assertEqual(departure_station((p['x'], p['y'], p['yaw']), STATIONS), key)

    def test_staging_is_not_dock(self):
        for key in ('station_1', 'station_2'):
            p = STATIONS[key]['target_pose']
            self.assertEqual(departure_station((p['x'], p['y'], p['yaw']), STATIONS), '')

    def test_mid_corridor_requires_recovery(self):
        with self.assertRaisesRegex(ValueError, 'RECOVERY_REQUIRED'):
            departure_station((2.974, -0.5805, -0.019), STATIONS)

    def test_angle_wrap(self):
        self.assertAlmostEqual(pose_error((0., 0., -math.pi + 0.01),
                                         {'x': 0., 'y': 0., 'yaw': math.pi - 0.01})[1], 0.02)


if __name__ == '__main__':
    unittest.main()
