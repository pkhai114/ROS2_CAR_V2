#!/usr/bin/env python3

import math
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class StationVisualizerNode(Node):
    """Publish HOME, station bodies, labels and docking lines for RViz2."""

    def __init__(self):
        super().__init__('station_visualizer_node')

        self.declare_parameter('station_file', '')
        self.declare_parameter('marker_topic', '/station_markers')
        self.declare_parameter('status_topic', '/station/status')
        self.declare_parameter('republish_period_sec', 2.0)

        station_file = str(self.get_parameter('station_file').value)
        marker_topic = str(self.get_parameter('marker_topic').value)
        status_topic = str(self.get_parameter('status_topic').value)
        republish_period = float(
            self.get_parameter('republish_period_sec').value
        )

        if not station_file:
            raise ValueError('Tham số station_file không được để trống.')

        self.frame_id, self.robot_front_offset, self.final_gap, self.stations = (
            self.load_station_file(station_file)
        )

        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.marker_publisher = self.create_publisher(
            MarkerArray,
            marker_topic,
            marker_qos,
        )

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.status_subscription = self.create_subscription(
            String,
            status_topic,
            self.status_callback,
            status_qos,
        )

        self.active_station = ''
        self.station_state = 'IDLE'

        self.publish_markers()

        if republish_period > 0.0:
            self.republish_timer = self.create_timer(
                republish_period,
                self.publish_markers,
            )

        self.get_logger().info(
            f'Đã nạp {len(self.stations)} vị trí từ {station_file}'
        )
        self.get_logger().info(
            f'Đang phát MarkerArray trên {marker_topic}, frame={self.frame_id}'
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
        robot_front_offset = float(system.get('robot_front_offset', 0.27))
        final_gap = float(system.get('final_gap', 0.10))

        if robot_front_offset < 0.0 or final_gap < 0.0:
            raise ValueError('robot_front_offset và final_gap không được âm.')

        raw_stations = data.get('stations')
        if not isinstance(raw_stations, dict) or not raw_stations:
            raise ValueError('stations.yaml không có mục stations hợp lệ.')

        stations = {}
        for key, raw in raw_stations.items():
            if not isinstance(raw, dict):
                raise ValueError(f'Cấu hình {key} không hợp lệ.')

            station_type = str(raw.get('type', 'station')).strip().lower()
            parsed = {
                'display_name': str(raw.get('display_name', key)),
                'type': station_type,
            }

            if station_type == 'home':
                parsed['pose'] = self.load_pose(raw['pose'], f'{key}.pose')
                size = raw.get('marker_size', {})
                parsed['length'] = self.require_number(
                    size, 'length', f'{key}.marker_size'
                )
                parsed['width'] = self.require_number(
                    size, 'width', f'{key}.marker_size'
                )
            else:
                parsed['staging_pose'] = self.load_pose(
                    raw['staging_pose'], f'{key}.staging_pose'
                )
                parsed['dock_pose'] = self.load_pose(
                    raw['dock_pose'], f'{key}.dock_pose'
                )
                size = raw.get('station_size', {})
                parsed['length'] = self.require_number(
                    size, 'length', f'{key}.station_size'
                )
                parsed['width'] = self.require_number(
                    size, 'width', f'{key}.station_size'
                )

            if parsed['length'] <= 0.0 or parsed['width'] <= 0.0:
                raise ValueError(f'Kích thước {key} phải lớn hơn 0.')

            stations[str(key)] = parsed

        return frame_id, robot_front_offset, final_gap, stations

    @staticmethod
    def yaw_to_quaternion(yaw):
        half_yaw = yaw / 2.0
        return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)

    def base_marker(self, marker_id, namespace, marker_type, stamp):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    @staticmethod
    def set_color(marker, red, green, blue, alpha=1.0):
        marker.color.r = float(red)
        marker.color.g = float(green)
        marker.color.b = float(blue)
        marker.color.a = float(alpha)

    def station_box_color(self, station_key):
        if station_key != self.active_station:
            return 0.0, 1.0, 0.0, 0.65

        if self.station_state in ('AT_STAGING', 'AT_HOME', 'DOCKED'):
            return 0.0, 0.65, 1.0, 0.85
        if self.station_state in ('FAILED', 'REJECTED', 'NAV2_UNAVAILABLE', 'CANCELED'):
            return 1.0, 0.0, 0.0, 0.85
        return 1.0, 0.55, 0.0, 0.85

    def make_box(self, marker_id, station_key, pose, length, width, stamp):
        marker = self.base_marker(
            marker_id, 'station_box', Marker.CUBE, stamp
        )
        marker.pose.position.x = pose['x']
        marker.pose.position.y = pose['y']
        marker.pose.position.z = 0.04

        qx, qy, qz, qw = self.yaw_to_quaternion(pose['yaw'])
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw

        marker.scale.x = length
        marker.scale.y = width
        marker.scale.z = 0.08
        self.set_color(marker, *self.station_box_color(station_key))
        return marker

    def make_label(self, marker_id, text, x, y, stamp):
        marker = self.base_marker(
            marker_id, 'station_label', Marker.TEXT_VIEW_FACING, stamp
        )
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.30
        marker.scale.z = 0.18
        marker.text = text
        self.set_color(marker, 1.0, 1.0, 1.0, 1.0)
        return marker

    def make_arrow(self, marker_id, pose, stamp):
        marker = self.base_marker(
            marker_id, 'dock_heading', Marker.ARROW, stamp
        )
        marker.pose.position.x = pose['x']
        marker.pose.position.y = pose['y']
        marker.pose.position.z = 0.10

        qx, qy, qz, qw = self.yaw_to_quaternion(pose['yaw'])
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw

        marker.scale.x = 0.35
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        self.set_color(marker, 1.0, 0.85, 0.0, 1.0)
        return marker

    def make_point(self, marker_id, namespace, pose, color, stamp):
        marker = self.base_marker(
            marker_id, namespace, Marker.SPHERE, stamp
        )
        marker.pose.position.x = pose['x']
        marker.pose.position.y = pose['y']
        marker.pose.position.z = 0.07
        marker.scale.x = 0.12
        marker.scale.y = 0.12
        marker.scale.z = 0.12
        self.set_color(marker, *color)
        return marker

    def make_docking_line(self, marker_id, staging, dock, stamp):
        marker = self.base_marker(
            marker_id, 'docking_line', Marker.LINE_STRIP, stamp
        )
        marker.scale.x = 0.045
        self.set_color(marker, 1.0, 1.0, 0.0, 1.0)

        start = Point()
        start.x = staging['x']
        start.y = staging['y']
        start.z = 0.06

        end = Point()
        end.x = dock['x']
        end.y = dock['y']
        end.z = 0.06

        marker.points = [start, end]
        return marker

    def physical_station_pose(self, station):
        dock = station['dock_pose']
        offset = (
            self.robot_front_offset
            + self.final_gap
            + station['length'] / 2.0
        )
        return {
            'x': dock['x'] + math.cos(dock['yaw']) * offset,
            'y': dock['y'] + math.sin(dock['yaw']) * offset,
            'yaw': dock['yaw'] + math.pi / 2.0,
        }

    def publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        delete_all = Marker()
        delete_all.header.frame_id = self.frame_id
        delete_all.header.stamp = stamp
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        for index, (key, station) in enumerate(self.stations.items()):
            base_id = index * 10

            if station['type'] == 'home':
                pose = station['pose']
                marker_array.markers.append(
                    self.make_box(
                        base_id,
                        key,
                        pose,
                        station['length'],
                        station['width'],
                        stamp,
                    )
                )
                marker_array.markers.append(
                    self.make_label(
                        base_id + 1,
                        station['display_name'],
                        pose['x'],
                        pose['y'],
                        stamp,
                    )
                )
                marker_array.markers.append(
                    self.make_arrow(base_id + 2, pose, stamp)
                )
                continue

            staging = station['staging_pose']
            dock = station['dock_pose']
            body_pose = self.physical_station_pose(station)

            marker_array.markers.append(
                self.make_box(
                    base_id,
                    key,
                    body_pose,
                    station['length'],
                    station['width'],
                    stamp,
                )
            )
            marker_array.markers.append(
                self.make_label(
                    base_id + 1,
                    station['display_name'],
                    body_pose['x'],
                    body_pose['y'],
                    stamp,
                )
            )
            marker_array.markers.append(
                self.make_docking_line(base_id + 2, staging, dock, stamp)
            )
            marker_array.markers.append(
                self.make_arrow(base_id + 3, dock, stamp)
            )
            marker_array.markers.append(
                self.make_point(
                    base_id + 4,
                    'staging_point',
                    staging,
                    (0.0, 0.45, 1.0, 1.0),
                    stamp,
                )
            )
            marker_array.markers.append(
                self.make_point(
                    base_id + 5,
                    'dock_point',
                    dock,
                    (1.0, 0.2, 0.0, 1.0),
                    stamp,
                )
            )

        self.marker_publisher.publish(marker_array)

    def status_callback(self, msg):
        text = msg.data.strip()
        if not text:
            return

        state, separator, station_key = text.partition(':')
        self.station_state = state.strip().upper()
        self.active_station = station_key.strip() if separator else ''
        self.publish_markers()


def main(args=None):
    rclpy.init(args=args)
    node = StationVisualizerNode()

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
