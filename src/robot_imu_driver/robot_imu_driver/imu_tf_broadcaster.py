#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class ImuTfBroadcaster(Node):
    def __init__(self):
        super().__init__('imu_tf_broadcaster')

        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'imu_link')

        self.imu_topic = self.get_parameter('imu_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value

        self.tf_broadcaster = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            10
        )

        self.get_logger().info(
            f'Broadcasting TF {self.parent_frame} -> {self.child_frame} from {self.imu_topic}'
        )

    def imu_callback(self, msg: Imu):
        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation = msg.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ImuTfBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
