#!/usr/bin/env python3
import sys
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)


CMD_VEL_TOPIC = "/cmd_vel"


class CmdVelRosNode(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_gui")

        self.publisher = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.subscription = self.create_subscription(
            Twist,
            CMD_VEL_TOPIC,
            self._cmd_vel_callback,
            10,
        )

        self.last_received_linear = 0.0
        self.last_received_angular = 0.0
        self.last_rx_monotonic: Optional[float] = None

        self.command_linear = 0.0
        self.command_angular = 0.0
        self.publish_enabled = False
        self.publish_rate_hz = 10

        self._publish_timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self._publish_timer_callback,
        )

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self.last_received_linear = float(msg.linear.x)
        self.last_received_angular = float(msg.angular.z)
        self.last_rx_monotonic = time.monotonic()

    def _publish_timer_callback(self) -> None:
        if not self.publish_enabled:
            return
        self.publish_current_command()

    def publish_current_command(self) -> None:
        msg = Twist()
        msg.linear.x = float(self.command_linear)
        msg.angular.z = float(self.command_angular)
        self.publisher.publish(msg)

    def set_command(self, linear_x: float, angular_z: float) -> None:
        self.command_linear = float(linear_x)
        self.command_angular = float(angular_z)

    def set_publish_enabled(self, enabled: bool) -> None:
        self.publish_enabled = enabled

    def set_publish_rate(self, rate_hz: int) -> None:
        rate_hz = max(1, min(int(rate_hz), 50))
        if rate_hz == self.publish_rate_hz:
            return

        self.publish_rate_hz = rate_hz
        self._publish_timer.cancel()
        self.destroy_timer(self._publish_timer)
        self._publish_timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self._publish_timer_callback,
        )


class CmdVelWindow(QMainWindow):
    def __init__(self, ros_node: CmdVelRosNode) -> None:
        super().__init__()
        self.ros_node = ros_node

        self.setWindowTitle("ROS 2 cmd_vel Control")
        self.resize(560, 430)

        self.current_linear_label = QLabel("0.000 m/s")
        self.current_angular_label = QLabel("0.000 rad/s")
        self.current_age_label = QLabel("Chưa nhận /cmd_vel")

        current_group = QGroupBox("Lệnh /cmd_vel hiện tại")
        current_form = QFormLayout()
        current_form.addRow("linear.x", self.current_linear_label)
        current_form.addRow("angular.z", self.current_angular_label)
        current_form.addRow("Trạng thái", self.current_age_label)
        current_group.setLayout(current_form)

        self.linear_input = QDoubleSpinBox()
        self.linear_input.setRange(-2.0, 2.0)
        self.linear_input.setDecimals(3)
        self.linear_input.setSingleStep(0.01)
        self.linear_input.setValue(0.15)
        self.linear_input.setSuffix(" m/s")
        self.linear_input.setKeyboardTracking(False)

        self.angular_input = QDoubleSpinBox()
        self.angular_input.setRange(-5.0, 5.0)
        self.angular_input.setDecimals(3)
        self.angular_input.setSingleStep(0.05)
        self.angular_input.setValue(0.0)
        self.angular_input.setSuffix(" rad/s")
        self.angular_input.setKeyboardTracking(False)

        self.rate_input = QSpinBox()
        self.rate_input.setRange(1, 50)
        self.rate_input.setValue(10)
        self.rate_input.setSuffix(" Hz")

        edit_group = QGroupBox("Lệnh mới")
        edit_form = QFormLayout()
        edit_form.addRow("linear.x", self.linear_input)
        edit_form.addRow("angular.z", self.angular_input)
        edit_form.addRow("Tần số publish", self.rate_input)
        edit_group.setLayout(edit_form)

        self.copy_button = QPushButton("Chép lệnh hiện tại")
        self.copy_button.clicked.connect(self._copy_current)

        self.start_button = QPushButton("Cập nhật và chạy liên tục")
        self.start_button.clicked.connect(self._start_or_update)

        self.publish_once_button = QPushButton("Publish một lần")
        self.publish_once_button.clicked.connect(self._publish_once)

        self.zero_button = QPushButton("Dừng robot (0, 0)")
        self.zero_button.clicked.connect(self._stop_robot)

        self.pause_button = QPushButton("Ngừng phát lệnh")
        self.pause_button.clicked.connect(self._pause_publishing)

        button_row_1 = QHBoxLayout()
        button_row_1.addWidget(self.copy_button)
        button_row_1.addWidget(self.publish_once_button)

        button_row_2 = QHBoxLayout()
        button_row_2.addWidget(self.start_button)
        button_row_2.addWidget(self.pause_button)

        self.status_label = QLabel("GUI chưa phát lệnh.")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setWordWrap(True)

        root_layout = QVBoxLayout()
        root_layout.addWidget(current_group)
        root_layout.addWidget(edit_group)
        root_layout.addLayout(button_row_1)
        root_layout.addLayout(button_row_2)
        root_layout.addWidget(self.zero_button)
        root_layout.addWidget(self.status_label)

        root = QWidget()
        root.setLayout(root_layout)
        self.setCentralWidget(root)

        self.gui_timer = QTimer(self)
        self.gui_timer.timeout.connect(self._spin_and_refresh)
        self.gui_timer.start(20)

    def _spin_and_refresh(self) -> None:
        rclpy.spin_once(self.ros_node, timeout_sec=0.0)

        self.current_linear_label.setText(
            f"{self.ros_node.last_received_linear:.3f} m/s"
        )
        self.current_angular_label.setText(
            f"{self.ros_node.last_received_angular:.3f} rad/s"
        )

        if self.ros_node.last_rx_monotonic is None:
            self.current_age_label.setText("Chưa nhận /cmd_vel")
        else:
            age = time.monotonic() - self.ros_node.last_rx_monotonic
            self.current_age_label.setText(f"Bản tin mới nhất cách đây {age:.2f} s")

        if self.ros_node.publish_enabled:
            self.status_label.setText(
                "Đang phát liên tục: "
                f"linear.x={self.ros_node.command_linear:.3f} m/s, "
                f"angular.z={self.ros_node.command_angular:.3f} rad/s, "
                f"{self.ros_node.publish_rate_hz} Hz"
            )

    def _read_inputs(self) -> tuple[float, float, int]:
        return (
            float(self.linear_input.value()),
            float(self.angular_input.value()),
            int(self.rate_input.value()),
        )

    def _copy_current(self) -> None:
        if self.ros_node.last_rx_monotonic is None:
            QMessageBox.warning(
                self,
                "Chưa có dữ liệu",
                "Chưa nhận được topic /cmd_vel để sao chép.",
            )
            return

        self.linear_input.setValue(self.ros_node.last_received_linear)
        self.angular_input.setValue(self.ros_node.last_received_angular)

    def _start_or_update(self) -> None:
        linear_x, angular_z, rate_hz = self._read_inputs()
        self.ros_node.set_publish_rate(rate_hz)
        self.ros_node.set_command(linear_x, angular_z)
        self.ros_node.set_publish_enabled(True)
        self.ros_node.publish_current_command()

    def _publish_once(self) -> None:
        linear_x, angular_z, rate_hz = self._read_inputs()
        self.ros_node.set_publish_rate(rate_hz)
        self.ros_node.set_command(linear_x, angular_z)
        self.ros_node.publish_current_command()
        self.status_label.setText(
            "Đã publish một lần. ESP32 watchdog sẽ dừng robot nếu không có nguồn "
            "khác tiếp tục publish /cmd_vel."
        )

    def _pause_publishing(self) -> None:
        self.ros_node.set_publish_enabled(False)
        self.status_label.setText(
            "GUI đã ngừng phát /cmd_vel. ESP32 watchdog sẽ đưa tốc độ về 0."
        )

    def _stop_robot(self) -> None:
        self.linear_input.setValue(0.0)
        self.angular_input.setValue(0.0)
        self.ros_node.set_command(0.0, 0.0)
        self.ros_node.set_publish_enabled(True)

        # Send immediately several times to make the stop command robust.
        for _ in range(3):
            self.ros_node.publish_current_command()

        self.status_label.setText(
            f"Đang phát lệnh dừng (0, 0) ở {self.ros_node.publish_rate_hz} Hz."
        )

    def closeEvent(self, event) -> None:
        # Send a final zero command before closing.
        self.ros_node.set_command(0.0, 0.0)
        for _ in range(3):
            self.ros_node.publish_current_command()

        self.gui_timer.stop()
        self.ros_node.destroy_node()
        rclpy.shutdown()
        event.accept()


def main(args=None) -> None:
    rclpy.init(args=args)
    ros_node = CmdVelRosNode()

    app = QApplication(sys.argv)
    window = CmdVelWindow(ros_node)
    window.show()

    exit_code = app.exec_()

    if rclpy.ok():
        ros_node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
