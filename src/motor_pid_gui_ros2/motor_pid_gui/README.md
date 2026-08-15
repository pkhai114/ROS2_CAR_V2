# motor_pid_gui

Giao diện PyQt5 độc lập cho ROS 2 Humble và firmware ESP32 PID hiện tại.

## Cài đặt

```bash
sudo apt update
sudo apt install python3-pyqt5
cp -r motor_pid_gui ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select motor_pid_gui
source install/setup.bash
ros2 run motor_pid_gui pid_tuning_gui
```

GUI đọc `/motor_pid/left/state`, `/motor_pid/right/state` và publish `/motor_pid/set`.
Sau khi gửi, GUI chờ ESP32 publish lại `data[13]..data[17]` để xác nhận thông số đã áp dụng.
