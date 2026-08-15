#!/usr/bin/env python3

import math
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import serial


class IM900DriverNode(Node):
    def __init__(self):
        super().__init__('imu_driver_node')

        self.declare_parameter('port', '/dev/ttyUSB_imu')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('report_rate_hz', 60)
        self.declare_parameter('report_tag', 0x0026)
        self.declare_parameter('compass_fusion', False)

        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.report_rate_hz = int(self.get_parameter('report_rate_hz').value)
        self.report_tag = int(self.get_parameter('report_tag').value)
        self.compass_fusion = bool(self.get_parameter('compass_fusion').value)

        self.pub = self.create_publisher(Imu, '/imu/data', 10)

        self.scale_accel = 0.00478515625
        self.scale_quat = 0.000030517578125
        self.scale_gyro_deg = 0.06103515625
        self.deg_to_rad = math.pi / 180.0

        self.rx_state = 0
        self.rx_buf = bytearray()
        self.rx_len = 0
        self.checksum = 0

        self.packet_count = 0

        self.get_logger().info(f'Opening IMU serial port: {self.port}, baudrate: {self.baudrate}')

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.01)
        except Exception as e:
            self.get_logger().error(f'Cannot open serial port {self.port}: {e}')
            raise

        time.sleep(0.2)
        self.configure_imu()
        time.sleep(0.2)

        self.timer = self.create_timer(0.001, self.read_serial)

        self.get_logger().info('IM900/IM948 IMU driver started')
        self.get_logger().info(f'Publishing sensor_msgs/Imu on /imu/data, frame_id={self.frame_id}')

    def configure_imu(self):
        acc_still = 5
        still_to_zero = 255
        move_to_zero = 0

        barometer_filter = 2
        gyro_filter = 1
        accel_filter = 3
        mag_filter = 5

        is_compass_on = 1 if self.compass_fusion else 0

        params = [0] * 11
        params[0] = 0x12
        params[1] = acc_still
        params[2] = still_to_zero
        params[3] = move_to_zero
        params[4] = ((barometer_filter & 0x03) << 1) | (is_compass_on & 0x01)
        params[5] = self.report_rate_hz
        params[6] = gyro_filter
        params[7] = accel_filter
        params[8] = mag_filter
        params[9] = self.report_tag & 0xFF
        params[10] = (self.report_tag >> 8) & 0xFF

        self.send_command(params)
        time.sleep(0.2)

        self.send_command([0x03])
        time.sleep(0.2)

        self.send_command([0x19])
        time.sleep(0.2)

        self.get_logger().info(
            f'IMU configured: report_tag=0x{self.report_tag:04X}, '
            f'rate={self.report_rate_hz}Hz, compass_fusion={self.compass_fusion}'
        )

    def send_command(self, data):
        dlen = len(data)
        if dlen == 0 or dlen > 19:
            self.get_logger().warn('Invalid command length')
            return

        preamble = bytes([0x00] * 46 + [0x00, 0xFF, 0x00, 0xFF])
        packet = bytearray()
        packet.append(0x49)
        packet.append(0xFF)
        packet.append(dlen)
        packet.extend(data)

        checksum = sum(packet[1:]) & 0xFF

        packet.append(checksum)
        packet.append(0x4D)

        self.ser.write(preamble + packet)

    def read_serial(self):
        try:
            data = self.ser.read(256)
        except Exception as e:
            self.get_logger().error(f'Serial read error: {e}')
            return

        for b in data:
            payload = self.feed_byte(b)
            if payload is not None:
                self.parse_payload(payload)

    def feed_byte(self, byte):
        if self.rx_state == 0:
            if byte == 0x49:
                self.rx_buf = bytearray([byte])
                self.checksum = 0
                self.rx_state = 1

        elif self.rx_state == 1:
            self.rx_buf.append(byte)
            self.checksum = byte
            if byte == 0xFF:
                self.rx_state = 0
            else:
                self.rx_state = 2

        elif self.rx_state == 2:
            self.rx_buf.append(byte)
            self.checksum = (self.checksum + byte) & 0xFF
            self.rx_len = byte
            if self.rx_len == 0 or self.rx_len > 73:
                self.rx_state = 0
            else:
                self.rx_state = 3

        elif self.rx_state == 3:
            self.rx_buf.append(byte)
            self.checksum = (self.checksum + byte) & 0xFF
            if len(self.rx_buf) >= self.rx_len + 3:
                self.rx_state = 4

        elif self.rx_state == 4:
            self.rx_buf.append(byte)
            if self.checksum == byte:
                self.rx_state = 5
            else:
                self.get_logger().warn('IMU packet checksum failed')
                self.rx_state = 0

        elif self.rx_state == 5:
            self.rx_buf.append(byte)
            self.rx_state = 0

            if byte == 0x4D:
                payload_start = 3
                payload_end = 3 + self.rx_len
                return bytes(self.rx_buf[payload_start:payload_end])

        else:
            self.rx_state = 0

        return None

    def parse_payload(self, payload):
        if len(payload) < 7:
            return

        if payload[0] != 0x11:
            return

        tag = payload[1] | (payload[2] << 8)

        idx = 7

        ax = ay = az = None
        gx = gy = gz = None
        qw = qx = qy = qz = None

        try:
            if tag & 0x0001:
                idx += 6

            if tag & 0x0002:
                ax_raw, ay_raw, az_raw = struct.unpack_from('<hhh', payload, idx)
                idx += 6
                ax = ax_raw * self.scale_accel
                ay = ay_raw * self.scale_accel
                az = az_raw * self.scale_accel

            if tag & 0x0004:
                gx_raw, gy_raw, gz_raw = struct.unpack_from('<hhh', payload, idx)
                idx += 6
                gx = gx_raw * self.scale_gyro_deg * self.deg_to_rad
                gy = gy_raw * self.scale_gyro_deg * self.deg_to_rad
                gz = gz_raw * self.scale_gyro_deg * self.deg_to_rad

            if tag & 0x0008:
                idx += 6

            if tag & 0x0010:
                idx += 8

            if tag & 0x0020:
                qw_raw, qx_raw, qy_raw, qz_raw = struct.unpack_from('<hhhh', payload, idx)
                idx += 8
                qw = qw_raw * self.scale_quat
                qx = qx_raw * self.scale_quat
                qy = qy_raw * self.scale_quat
                qz = qz_raw * self.scale_quat

        except struct.error:
            self.get_logger().warn('Payload parse error')
            return

        if ax is None or gx is None or qw is None:
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.orientation.x = float(qx)
        msg.orientation.y = float(qy)
        msg.orientation.z = float(qz)
        msg.orientation.w = float(qw)

        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)

        msg.linear_acceleration.x = float(ax)
        msg.linear_acceleration.y = float(ay)
        msg.linear_acceleration.z = float(az)

        msg.orientation_covariance = [
            0.05, 0.0, 0.0,
            0.0, 0.05, 0.0,
            0.0, 0.0, 0.08,
        ]

        msg.angular_velocity_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.02,
        ]

        msg.linear_acceleration_covariance = [
            0.20, 0.0, 0.0,
            0.0, 0.20, 0.0,
            0.0, 0.0, 0.30,
        ]

        self.pub.publish(msg)

        self.packet_count += 1
        if self.packet_count % 300 == 0:
            self.get_logger().info(
                f'IMU OK: q=[{qx:+.3f}, {qy:+.3f}, {qz:+.3f}, {qw:+.3f}], '
                f'g_z={gz:+.5f} rad/s, acc_z={az:+.3f} m/s^2'
            )

    def destroy_node(self):
        try:
            if hasattr(self, 'ser') and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IM900DriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
