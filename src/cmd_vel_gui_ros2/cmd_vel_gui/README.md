# cmd_vel_gui

Node ROS 2 Python + PyQt5 độc lập để:

- Hiển thị `linear.x` và `angular.z` hiện tại trên `/cmd_vel`
- Nhập lệnh mới
- Cập nhật lệnh khi robot đang chạy
- Publish liên tục ở tần số lựa chọn
- Gửi lệnh dừng `(0, 0)`
- Ngừng phát để ESP32 watchdog tự xử lý

## Cài đặt

```bash
sudo apt update
sudo apt install python3-pyqt5
```

Chép package vào workspace:

```bash
cp -r cmd_vel_gui ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select cmd_vel_gui
source install/setup.bash
```

## Chạy

```bash
ros2 run cmd_vel_gui cmd_vel_gui
```

Không chạy đồng thời lệnh terminal `ros2 topic pub /cmd_vel ... -r 10`,
vì hai publisher sẽ tranh nhau điều khiển robot.
