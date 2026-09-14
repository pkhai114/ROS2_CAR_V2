"""ROS-independent station sequencing. All motion is requested via effects."""

import math
import uuid


def pose_error(actual, target):
    angle = target['yaw'] - actual[2]
    return (math.hypot(target['x'] - actual[0], target['y'] - actual[1]),
            abs(math.atan2(math.sin(angle), math.cos(angle))))


def departure_station(actual, stations, dock_radius=0.14, corridor_width=0.18):
    """Conservative boot-time check: never navigate out of a docking corridor."""
    near = [key for key, st in stations.items() if st['type'] == 'station'
            and pose_error(actual, st['dock_pose'])[0] <= dock_radius]
    if len(near) > 1:
        raise ValueError('AMBIGUOUS_DOCK_POSITION')
    if near:
        return near[0]
    for st in stations.values():
        if st['type'] != 'station':
            continue
        a, b = st['target_pose'], st['dock_pose']
        dx, dy = b['x'] - a['x'], b['y'] - a['y']
        length = math.hypot(dx, dy)
        if length < 0.10:
            raise ValueError('INVALID_DOCKING_PATH')
        px, py = actual[0] - a['x'], actual[1] - a['y']
        along = (px * dx + py * dy) / length
        lateral = (-dy * px + dx * py) / length
        if 0.10 < along < length + 0.27 and abs(lateral) < corridor_width:
            raise ValueError('RECOVERY_REQUIRED_IN_DOCKING_CORRIDOR')
    return ''


class StationWorkflow:
    """One command at a time; correlated results; explicit completion gates."""

    RESTING = {'IDLE', 'DOCKED', 'AT_HOME', 'CANCELED', 'FAILED'}

    def __init__(self, stations, handoff_timeout=5.0, settle_timeout=8.0,
                 navigation_timeout=180.0, docking_timeout=35.0):
        self.stations = stations
        self.handoff_timeout = handoff_timeout
        self.settle_timeout = settle_timeout
        self.navigation_timeout = navigation_timeout
        self.docking_timeout = docking_timeout
        self.state = 'IDLE'
        self.target = ''
        self.docked_station = ''
        self.departing = ''
        self.request_id = ''
        self.nav_token = ''
        self.dock_started = False
        self.deadline = math.inf
        self.effects = []
        self.failure_reason = ''
        self.cancel_is_failure = False

    @property
    def busy(self):
        return self.state not in self.RESTING

    def emit(self, kind, **data):
        self.effects.append((kind, data))

    def drain(self):
        effects, self.effects = self.effects, []
        return effects

    def transition(self, state, now, timeout=None):
        self.state = state
        self.deadline = math.inf if timeout is None else now + timeout
        self.emit('status', state=state, station=self.target)

    def start(self, station, departure, now):
        if station not in self.stations:
            return 'INVALID_STATION'
        if self.busy:
            return 'ALREADY_RUNNING' if station == self.target else 'BUSY'
        if self.state in ('FAILED', 'CANCELED'):
            return 'RESET_REQUIRED'
        if departure == station:
            self.docked_station = departure
            self.target = station
            self.transition('CHECK_DOCK', now, self.settle_timeout)
            self.emit('mode', mode='HOLD')
            return 'VERIFY_ALREADY_DOCKED'
        self.target, self.departing = station, departure
        self.docked_station = departure
        self.failure_reason = ''
        self.request_id = self.nav_token = ''
        self.transition('HOLD_BEFORE_UNDOCK' if departure else 'HOLD_BEFORE_NAV',
                        now, self.handoff_timeout)
        self.emit('mode', mode='HOLD')
        return 'ACCEPTED'

    def hold_ready(self, now):
        if self.state == 'HOLD_BEFORE_UNDOCK':
            self.begin_docking('undock', self.departing, now)
        elif self.state == 'HOLD_BEFORE_NAV':
            self.transition('WAIT_NAV_MODE', now, self.handoff_timeout)
            self.emit('mode', mode='NAV2')

    def nav_mode_ready(self, now):
        if self.state != 'WAIT_NAV_MODE':
            return
        self.nav_token = uuid.uuid4().hex
        self.transition('SENDING_GOAL', now, self.handoff_timeout)
        self.emit('navigate', station=self.target, token=self.nav_token)

    def nav_accepted(self, token, accepted, now):
        if token != self.nav_token or self.state != 'SENDING_GOAL':
            return
        if accepted:
            self.transition('NAVIGATING', now, self.navigation_timeout)
        else:
            self.fail('NAV2_REJECTED', now)

    def nav_result(self, token, succeeded, now):
        if token != self.nav_token or self.state != 'NAVIGATING':
            return
        if not succeeded:
            self.fail('NAV2_FAILED', now)
            return
        self.transition('CHECK_STAGING', now, self.settle_timeout)
        self.emit('mode', mode='HOLD')

    def settled(self, now):
        if self.state == 'CHECK_STAGING':
            if self.stations[self.target]['type'] == 'home':
                self.transition('AT_HOME', now)
            else:
                self.emit('status', state='AT_STAGING', station=self.target)
                self.begin_docking('dock', self.target, now)
        elif self.state == 'CHECK_UNDOCK':
            self.docked_station = ''
            self.departing = ''
            self.transition('HOLD_BEFORE_NAV', now, self.handoff_timeout)
            self.emit('mode', mode='HOLD')
        elif self.state == 'CHECK_DOCK':
            self.docked_station = self.target
            self.transition('DOCKED', now)

    def begin_docking(self, operation, station, now):
        self.request_id = uuid.uuid4().hex
        self.dock_started = False
        self.transition('WAIT_UNDOCK_ACK' if operation == 'undock' else 'WAIT_DOCK_ACK',
                        now, self.handoff_timeout)
        self.emit('docking', operation=operation, station=station,
                  request_id=self.request_id)

    def docking_report(self, report, now):
        if report.get('request_id') != self.request_id or not self.request_id:
            return
        if self.state not in ('WAIT_DOCK_ACK', 'DOCKING',
                              'WAIT_UNDOCK_ACK', 'UNDOCKING'):
            return
        reverse = self.state in ('WAIT_UNDOCK_ACK', 'UNDOCKING')
        station = self.departing if reverse else self.target
        if report.get('station') != station:
            self.fail('DOCKING_STATION_MISMATCH', now)
            return
        status = report.get('status', '')
        prefix = 'UNDOCK_' if reverse else 'DOCK_'
        if status.startswith(prefix):
            if not self.dock_started:
                self.dock_started = True
                self.transition('UNDOCKING' if reverse else 'DOCKING',
                                now, self.docking_timeout)
            return
        if status == ('UNDOCKED' if reverse else 'DOCKED'):
            if not self.dock_started:
                self.fail('DOCKING_RESULT_WITHOUT_ACK', now)
                return
            self.transition('CHECK_UNDOCK' if reverse else 'CHECK_DOCK',
                            now, self.settle_timeout)
            self.emit('mode', mode='HOLD')
            return
        self.fail('DOCKING_' + status, now)

    def cancel(self, now, reason='USER_CANCEL', failure=False):
        if self.state == 'CANCELING':
            return
        self.cancel_is_failure = failure
        self.failure_reason = reason
        self.emit('mode', mode='HOLD')
        self.emit('cancel_children', token=self.nav_token,
                  request_id=self.request_id)
        self.transition('CANCELING', now, self.handoff_timeout)

    def fail(self, reason, now):
        self.cancel(now, reason, failure=True)

    def children_stopped(self, now):
        if self.state == 'CANCELING':
            self.transition('FAILED' if self.cancel_is_failure else 'CANCELED', now)

    def tick(self, now):
        if now <= self.deadline:
            return
        if self.state == 'CANCELING':
            self.failure_reason += ':CANCEL_TIMEOUT'
            self.transition('FAILED', now)
            self.emit('mode', mode='FAULT')
        else:
            self.fail('TIMEOUT_' + self.state, now)

    def reset(self, now):
        if self.busy:
            return False
        self.target = self.departing = self.request_id = self.nav_token = ''
        self.failure_reason = ''
        self.transition('IDLE', now)
        self.emit('mode', mode='HOLD')
        return True
