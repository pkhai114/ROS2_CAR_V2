import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanRangeFilter(Node):
    """Remove scan returns that are closer than the configured distance."""

    def __init__(self):
        super().__init__('scan_range_filter')

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan/filtered')
        self.declare_parameter('min_distance', 0.30)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.min_distance = float(
            self.get_parameter('min_distance').value
        )

        if self.min_distance <= 0.0:
            raise ValueError('min_distance must be greater than zero')

        self.publisher = self.create_publisher(
            LaserScan,
            output_topic,
            QoSProfile(depth=10),
        )
        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Filtering {input_topic} -> {output_topic}; '
            f'discarding ranges below {self.min_distance:.2f} m'
        )

    def scan_callback(self, scan):
        filtered_scan = LaserScan()
        filtered_scan.header = scan.header
        filtered_scan.angle_min = scan.angle_min
        filtered_scan.angle_max = scan.angle_max
        filtered_scan.angle_increment = scan.angle_increment
        filtered_scan.time_increment = scan.time_increment
        filtered_scan.scan_time = scan.scan_time
        filtered_scan.range_min = max(scan.range_min, self.min_distance)
        filtered_scan.range_max = scan.range_max
        filtered_scan.ranges = [
            math.inf
            if math.isfinite(distance) and distance < self.min_distance
            else distance
            for distance in scan.ranges
        ]
        filtered_scan.intensities = list(scan.intensities)

        self.publisher.publish(filtered_scan)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRangeFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
