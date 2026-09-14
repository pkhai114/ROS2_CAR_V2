#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


class HomeInitialPoseNode(Node):
    """
    Tự động gửi tọa độ HOME cho AMCL đúng một lần.

    Chỉ chạy node khi robot đang được đặt vật lý tại HOME.
    Node không điều khiển robot di chuyển về HOME.
    """

    def __init__(self):
        super().__init__('home_initial_pose_node')

        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('home_yaw', 0.0)

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('output_topic', '/initialpose')

        # Chờ thêm sau khi phát hiện AMCL đã subscribe.
        self.declare_parameter('publish_delay_sec', 1.0)

        # Đây là phương sai, không phải độ lệch chuẩn.
        self.declare_parameter('covariance_x', 0.01)
        self.declare_parameter('covariance_y', 0.01)
        self.declare_parameter('covariance_yaw', 0.0076)

        self.home_x = float(
            self.get_parameter('home_x').value
        )
        self.home_y = float(
            self.get_parameter('home_y').value
        )
        self.home_yaw = float(
            self.get_parameter('home_yaw').value
        )

        self.frame_id = str(
            self.get_parameter('frame_id').value
        )
        self.output_topic = str(
            self.get_parameter('output_topic').value
        )
        self.publish_delay_sec = float(
            self.get_parameter('publish_delay_sec').value
        )

        self.covariance_x = float(
            self.get_parameter('covariance_x').value
        )
        self.covariance_y = float(
            self.get_parameter('covariance_y').value
        )
        self.covariance_yaw = float(
            self.get_parameter('covariance_yaw').value
        )

        values = [
            self.home_x,
            self.home_y,
            self.home_yaw,
            self.publish_delay_sec,
            self.covariance_x,
            self.covariance_y,
            self.covariance_yaw,
        ]

        if not all(math.isfinite(value) for value in values):
            raise ValueError(
                'Các tham số HOME phải là số hữu hạn.'
            )

        if self.publish_delay_sec < 0.0:
            raise ValueError(
                'publish_delay_sec không được âm.'
            )

        if (
            self.covariance_x < 0.0
            or self.covariance_y < 0.0
            or self.covariance_yaw < 0.0
        ):
            raise ValueError(
                'Các giá trị covariance không được âm.'
            )

        # Chuẩn hóa góc về [-pi, pi].
        self.home_yaw = math.atan2(
            math.sin(self.home_yaw),
            math.cos(self.home_yaw),
        )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.output_topic,
            qos,
        )

        self.initial_pose_sent = False
        self.subscriber_detected_time = None

        # Kiểm tra AMCL mỗi 0,2 giây.
        self.timer = self.create_timer(
            0.2,
            self.try_publish_home_pose,
        )

        self.get_logger().info(
            'Đang chờ AMCL subscribe topic '
            f'{self.output_topic}...'
        )

        self.get_logger().info(
            'HOME đã cấu hình: '
            f'x={self.home_x:.3f} m, '
            f'y={self.home_y:.3f} m, '
            f'yaw={self.home_yaw:.6f} rad '
            f'({math.degrees(self.home_yaw):.2f} độ)'
        )

    def try_publish_home_pose(self):
        if self.initial_pose_sent:
            return

        # Chưa có AMCL hoặc subscriber nào trên /initialpose.
        if self.initial_pose_publisher.get_subscription_count() == 0:
            self.subscriber_detected_time = None
            return

        now = self.get_clock().now()

        if self.subscriber_detected_time is None:
            self.subscriber_detected_time = now

            self.get_logger().info(
                'Đã phát hiện subscriber trên /initialpose. '
                f'Chờ thêm {self.publish_delay_sec:.1f} giây.'
            )
            return

        elapsed_sec = (
            now - self.subscriber_detected_time
        ).nanoseconds / 1e9

        if elapsed_sec < self.publish_delay_sec:
            return

        msg = PoseWithCovarianceStamped()

        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id

        msg.pose.pose.position.x = self.home_x
        msg.pose.pose.position.y = self.home_y
        msg.pose.pose.position.z = 0.0

        half_yaw = self.home_yaw / 2.0

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(half_yaw)
        msg.pose.pose.orientation.w = math.cos(half_yaw)

        # Thứ tự covariance:
        # x, y, z, roll, pitch, yaw
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = self.covariance_x
        msg.pose.covariance[7] = self.covariance_y
        msg.pose.covariance[35] = self.covariance_yaw

        self.initial_pose_publisher.publish(msg)
        self.initial_pose_sent = True
        self.timer.cancel()

        self.get_logger().info(
            'Đã gửi HOME Initial Pose cho AMCL: '
            f'x={self.home_x:.3f} m, '
            f'y={self.home_y:.3f} m, '
            f'yaw={self.home_yaw:.6f} rad '
            f'({math.degrees(self.home_yaw):.2f} độ)'
        )

        self.get_logger().info(
            'Node đã khóa và sẽ không gửi Initial Pose lần hai.'
        )


def main(args=None):
    rclpy.init(args=args)

    node = HomeInitialPoseNode()

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