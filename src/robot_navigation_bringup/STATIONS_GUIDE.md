# Hệ thống trạm ROS2 - phiên bản 1

Phiên bản này gồm hai node:

- `station_visualizer_node.py`: hiển thị HOME, thân trạm, nhãn, staging pose,
  dock pose và đường docking trên RViz2.
- `station_manager_node.py`: nhận tên trạm và gửi Nav2
  `NavigateToPose` tới HOME hoặc `staging_pose`.

Phiên bản này chưa tự chạy từ staging tới dock.

## Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_navigation_bringup --symlink-install
source install/setup.bash
```

## Chạy

Chạy Nav2 trước. Sau khi `/navigate_to_pose` sẵn sàng:

```bash
ros2 launch robot_navigation_bringup stations.launch.py
```

## Hiển thị trong RViz2

Thêm display:

```text
Add -> By display type -> MarkerArray
Topic: /station_markers
Reliability Policy: Reliable
Durability Policy: Transient Local
```

Các marker:

- Khối xanh: HOME hoặc thân trạm vật lý.
- Chấm xanh dương: staging pose.
- Chấm đỏ: dock pose.
- Đường vàng: đoạn staging tới dock.
- Mũi tên vàng: hướng đầu xe khi docking.

## Ra lệnh

Trạm 1:

```bash
ros2 topic pub --once /station/command std_msgs/msg/String \
  "{data: 'station_1'}"
```

Trạm 2:

```bash
ros2 topic pub --once /station/command std_msgs/msg/String \
  "{data: 'station_2'}"
```

HOME:

```bash
ros2 topic pub --once /station/command std_msgs/msg/String \
  "{data: 'home'}"
```

Hủy goal đang chạy:

```bash
ros2 topic pub --once /station/command std_msgs/msg/String \
  "{data: 'cancel'}"
```

## Theo dõi

```bash
ros2 topic echo /station/status
```

```bash
ros2 topic echo /station/current
```

```bash
ros2 topic echo /station/distance_remaining
```

Các trạng thái chính:

- `IDLE`
- `SENDING_GOAL:<station>`
- `NAVIGATING:<station>`
- `AT_STAGING:<station>`
- `AT_HOME:home`
- `CANCELED:<station>`
- `FAILED:<station>`

## An toàn

`home` trong `station_manager_node.py` là goal điều hướng để xe chạy về HOME.
Nó khác với `home_initial_pose_node.py`, node chỉ khai báo vị trí ban đầu cho
AMCL khi xe đang được đặt vật lý tại HOME.
