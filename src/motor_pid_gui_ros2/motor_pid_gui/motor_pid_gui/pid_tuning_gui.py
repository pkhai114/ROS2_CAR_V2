#!/usr/bin/env python3
import math
import sys
import time
from dataclasses import dataclass
from typing import Optional, Sequence

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QDoubleSpinBox, QFormLayout, QGridLayout,
    QGroupBox, QHBoxLayout, QLabel, QMainWindow, QMessageBox,
    QPushButton, QVBoxLayout, QWidget,
)

LEFT_STATE_TOPIC = "/motor_pid/left/state"
RIGHT_STATE_TOPIC = "/motor_pid/right/state"
PID_SET_TOPIC = "/motor_pid/set"

IDX_KP = 13
IDX_KI = 14
IDX_KD = 15
IDX_KFF = 16
IDX_ALPHA = 17
MIN_STATE_LENGTH = 18

MOTOR_LEFT = 0.0
MOTOR_RIGHT = 1.0


@dataclass
class PidValues:
    kp: float
    ki: float
    kd: float
    kff: float
    alpha: float

    def as_list(self):
        return [self.kp, self.ki, self.kd, self.kff, self.alpha]

    def almost_equal(self, other: "PidValues", tolerance: float = 1e-4) -> bool:
        return all(abs(a - b) <= tolerance for a, b in zip(self.as_list(), other.as_list()))


class MotorPidRosNode(Node):
    def __init__(self):
        super().__init__("motor_pid_gui")
        self.pid_publisher = self.create_publisher(Float32MultiArray, PID_SET_TOPIC, 10)
        self.create_subscription(Float32MultiArray, LEFT_STATE_TOPIC, self._left_cb, 10)
        self.create_subscription(Float32MultiArray, RIGHT_STATE_TOPIC, self._right_cb, 10)

        self.left_values: Optional[PidValues] = None
        self.right_values: Optional[PidValues] = None
        self.left_last_rx: Optional[float] = None
        self.right_last_rx: Optional[float] = None
        self.left_pending: Optional[PidValues] = None
        self.right_pending: Optional[PidValues] = None

    @staticmethod
    def _parse(data: Sequence[float]) -> Optional[PidValues]:
        if len(data) < MIN_STATE_LENGTH:
            return None
        vals = [float(data[i]) for i in (IDX_KP, IDX_KI, IDX_KD, IDX_KFF, IDX_ALPHA)]
        if not all(math.isfinite(v) for v in vals):
            return None
        return PidValues(*vals)

    def _left_cb(self, msg):
        parsed = self._parse(msg.data)
        if parsed is None:
            self.get_logger().warning(f"Left state invalid: received {len(msg.data)} values")
            return
        self.left_values = parsed
        self.left_last_rx = time.monotonic()
        if self.left_pending and parsed.almost_equal(self.left_pending):
            self.left_pending = None

    def _right_cb(self, msg):
        parsed = self._parse(msg.data)
        if parsed is None:
            self.get_logger().warning(f"Right state invalid: received {len(msg.data)} values")
            return
        self.right_values = parsed
        self.right_last_rx = time.monotonic()
        if self.right_pending and parsed.almost_equal(self.right_pending):
            self.right_pending = None

    def publish_pid(self, motor_id: float, values: PidValues, reset_state: bool):
        msg = Float32MultiArray()
        msg.data = [
            motor_id, values.kp, values.ki, values.kd,
            values.kff, values.alpha, 1.0 if reset_state else 0.0,
        ]
        self.pid_publisher.publish(msg)
        if motor_id == MOTOR_LEFT:
            self.left_pending = values
        else:
            self.right_pending = values


class MotorPanel(QGroupBox):
    def __init__(self, title, send_callback, copy_callback):
        super().__init__(title)
        self.current = {name: QLabel("—") for name in ("kp", "ki", "kd", "kff", "alpha")}
        self.inputs = {
            "kp": self._spin(0.0, 50.0, 0.9, 0.01, 4),
            "ki": self._spin(0.0, 100.0, 1.1, 0.01, 4),
            "kd": self._spin(0.0, 20.0, 0.0, 0.01, 5),
            "kff": self._spin(0.0, 10.0, 1.1, 0.01, 4),
            "alpha": self._spin(0.0, 1.0, 0.3, 0.01, 3),
        }

        current_box = QGroupBox("PID hiện tại trên ESP32")
        current_form = QFormLayout(current_box)
        current_form.addRow("Kp", self.current["kp"])
        current_form.addRow("Ki", self.current["ki"])
        current_form.addRow("Kd", self.current["kd"])
        current_form.addRow("Kff", self.current["kff"])
        current_form.addRow("D filter alpha", self.current["alpha"])

        edit_box = QGroupBox("Thông số mới")
        edit_form = QFormLayout(edit_box)
        edit_form.addRow("Kp", self.inputs["kp"])
        edit_form.addRow("Ki", self.inputs["ki"])
        edit_form.addRow("Kd", self.inputs["kd"])
        edit_form.addRow("Kff", self.inputs["kff"])
        edit_form.addRow("D filter alpha", self.inputs["alpha"])

        self.reset_checkbox = QCheckBox("Reset trạng thái I và D sau khi cập nhật")
        self.reset_checkbox.setChecked(True)

        copy_btn = QPushButton("Chép PID hiện tại")
        copy_btn.clicked.connect(copy_callback)
        send_btn = QPushButton("Gửi PID xuống ESP32")
        send_btn.clicked.connect(send_callback)

        buttons = QHBoxLayout()
        buttons.addWidget(copy_btn)
        buttons.addWidget(send_btn)

        self.status = QLabel("Chưa nhận dữ liệu")
        self.status.setWordWrap(True)

        layout = QVBoxLayout(self)
        layout.addWidget(current_box)
        layout.addWidget(edit_box)
        layout.addWidget(self.reset_checkbox)
        layout.addLayout(buttons)
        layout.addWidget(self.status)

    @staticmethod
    def _spin(minimum, maximum, value, step, decimals):
        box = QDoubleSpinBox()
        box.setRange(minimum, maximum)
        box.setValue(value)
        box.setSingleStep(step)
        box.setDecimals(decimals)
        box.setKeyboardTracking(False)
        return box

    def get_values(self):
        return PidValues(
            self.inputs["kp"].value(), self.inputs["ki"].value(),
            self.inputs["kd"].value(), self.inputs["kff"].value(),
            self.inputs["alpha"].value(),
        )

    def set_inputs(self, v: PidValues):
        self.inputs["kp"].setValue(v.kp)
        self.inputs["ki"].setValue(v.ki)
        self.inputs["kd"].setValue(v.kd)
        self.inputs["kff"].setValue(v.kff)
        self.inputs["alpha"].setValue(v.alpha)

    def set_current(self, v: Optional[PidValues]):
        if v is None:
            for label in self.current.values():
                label.setText("—")
            return
        self.current["kp"].setText(f"{v.kp:.4f}")
        self.current["ki"].setText(f"{v.ki:.4f}")
        self.current["kd"].setText(f"{v.kd:.5f}")
        self.current["kff"].setText(f"{v.kff:.4f}")
        self.current["alpha"].setText(f"{v.alpha:.3f}")


class MotorPidWindow(QMainWindow):
    def __init__(self, node: MotorPidRosNode):
        super().__init__()
        self.node = node
        self.setWindowTitle("ESP32 Motor PID Tuning")
        self.resize(920, 560)

        self.left = MotorPanel("Động cơ trái", self._send_left, self._copy_left)
        self.right = MotorPanel("Động cơ phải", self._send_right, self._copy_right)

        self.connection = QLabel("ROS 2: đang chờ dữ liệu từ ESP32...")
        self.connection.setAlignment(Qt.AlignCenter)
        hint = QLabel("Đọc: /motor_pid/left/state, /motor_pid/right/state    |    Gửi: /motor_pid/set")
        hint.setAlignment(Qt.AlignCenter)

        grid = QGridLayout()
        grid.addWidget(self.left, 0, 0)
        grid.addWidget(self.right, 0, 1)

        root_layout = QVBoxLayout()
        root_layout.addWidget(self.connection)
        root_layout.addLayout(grid)
        root_layout.addWidget(hint)
        root = QWidget()
        root.setLayout(root_layout)
        self.setCentralWidget(root)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(20)

    def _tick(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.left.set_current(self.node.left_values)
        self.right.set_current(self.node.right_values)

        now = time.monotonic()
        left_age = None if self.node.left_last_rx is None else now - self.node.left_last_rx
        right_age = None if self.node.right_last_rx is None else now - self.node.right_last_rx

        if left_age is not None and right_age is not None and left_age < 1.0 and right_age < 1.0:
            self.connection.setText(f"ESP32 đang cập nhật — trái {left_age:.2f}s, phải {right_age:.2f}s")
        else:
            self.connection.setText("Chưa nhận đủ dữ liệu PID hoặc dữ liệu đã quá 1 giây")

        self._status(self.left, self.node.left_pending, left_age)
        self._status(self.right, self.node.right_pending, right_age)

    @staticmethod
    def _status(panel, pending, age):
        if age is None:
            panel.status.setText("Chưa nhận state topic")
        elif age >= 1.0:
            panel.status.setText(f"Mất cập nhật state — bản tin cuối cách đây {age:.2f}s")
        elif pending is not None:
            panel.status.setText("Đã gửi PID; đang chờ ESP32 publish lại để xác nhận")
        else:
            panel.status.setText(f"Đã đồng bộ với ESP32 — state mới nhất {age:.2f}s")

    def _copy_left(self):
        if self.node.left_values is None:
            QMessageBox.warning(self, "Chưa có dữ liệu", "Chưa nhận PID động cơ trái")
            return
        self.left.set_inputs(self.node.left_values)

    def _copy_right(self):
        if self.node.right_values is None:
            QMessageBox.warning(self, "Chưa có dữ liệu", "Chưa nhận PID động cơ phải")
            return
        self.right.set_inputs(self.node.right_values)

    def _send_left(self):
        self.node.publish_pid(MOTOR_LEFT, self.left.get_values(), self.left.reset_checkbox.isChecked())
        self.left.status.setText("Đã publish PID mới cho động cơ trái")

    def _send_right(self):
        self.node.publish_pid(MOTOR_RIGHT, self.right.get_values(), self.right.reset_checkbox.isChecked())
        self.right.status.setText("Đã publish PID mới cho động cơ phải")

    def closeEvent(self, event):
        self.timer.stop()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        event.accept()


def main(args=None):
    rclpy.init(args=args)
    node = MotorPidRosNode()
    app = QApplication(sys.argv)
    window = MotorPidWindow(node)
    window.show()
    code = app.exec_()
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
