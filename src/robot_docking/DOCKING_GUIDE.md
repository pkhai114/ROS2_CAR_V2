# Cài đặt và thử nghiệm robot_docking

Package này thực hiện hai thao tác:

- `DOCK`: từ `staging_pose`, căn yaw, bám đường thẳng bằng lookahead + PD,
  căn yaw cuối và xác nhận pose cùng khoảng cách LiDAR.
- `UNDOCK`: từ `dock_pose`, lùi theo chính đường thẳng đó tới `staging_pose`,
  đồng thời giám sát vật cản phía sau.

`velocity_arbiter` là node duy nhất được phép phát `/cmd_vel` tới ESP32.
Không chạy thử xe nếu bước kiểm tra publisher ở mục 4 chưa đạt.

## 1. Tạo cấu trúc thư mục

```bash
cd ~/ros2_ws/src

mkdir -p robot_docking/scripts
mkdir -p robot_docking/config
mkdir -p robot_docking/launch
```

Đặt từng file vào đúng vị trí:

```text
~/ros2_ws/src/robot_docking/package.xml
~/ros2_ws/src/robot_docking/CMakeLists.txt
~/ros2_ws/src/robot_docking/DOCKING_GUIDE.md
~/ros2_ws/src/robot_docking/scripts/velocity_arbiter_node.py
~/ros2_ws/src/robot_docking/scripts/docking_controller_node.py
~/ros2_ws/src/robot_docking/config/docking_params.yaml
~/ros2_ws/src/robot_docking/launch/docking.launch.py
```

Thay file launch Nav2 bằng bản đã sửa:

```text
~/ros2_ws/src/robot_navigation_bringup/launch/navigation.launch.py
```

Không đổi `stations.yaml`; docking dùng trực tiếp file hiện tại của
`robot_navigation_bringup`.

## 2. Cấp quyền và build

```bash
chmod +x \
  ~/ros2_ws/src/robot_docking/scripts/velocity_arbiter_node.py \
  ~/ros2_ws/src/robot_docking/scripts/docking_controller_node.py

cd ~/ros2_ws

source /opt/ros/humble/setup.bash

colcon build \
  --packages-select robot_navigation_bringup robot_docking \
  --symlink-install

source install/setup.bash
```

## 3. Khởi động

Giữ toàn bộ nguồn dữ liệu hiện có: ESP32, LiDAR `/scan/filtered`, EKF
`/odometry/filtered`, localization và TF. TF LiDAR phải có đúng một nguồn:

```text
base_link -> laser
x=0.0, y=0.0, z=0.22, yaw=-1.5708
```

Khởi động arbiter và docking trước Nav2:

```bash
ros2 launch robot_docking docking.launch.py
```

Terminal khác, chạy Nav2 với map đang sử dụng:

```bash
ros2 launch robot_navigation_bringup navigation.launch.py \
  map:=/home/t9/maps/map_moi/robot_map.yaml
```

Terminal khác, chạy hệ thống trạm:

```bash
ros2 launch robot_navigation_bringup stations.launch.py
```

Nếu map mới đã là map mặc định thì bỏ đối số `map:=...`.

## 4. Kiểm tra luồng vận tốc — bắt buộc

```bash
ros2 node info /velocity_smoother
```

Kết quả mới phải là:

```text
Subscriber: /cmd_vel_nav
Publisher:  /cmd_vel_nav_output
```

Kiểm tra đầu ra cuối:

```bash
ros2 topic info /cmd_vel -v
```

Điều kiện đạt:

```text
Publisher count: 1
Node name: velocity_arbiter

Subscription count: 1
Node name: esp32_base_controller
```

Các publisher Nav2 cũ phải được chuyển sang nhánh riêng:

```bash
ros2 topic info /cmd_vel_nav_output -v
```

Nếu `/cmd_vel` vẫn có `behavior_server` hoặc `velocity_smoother` phát trực tiếp,
không đặt xe xuống đất và không gửi lệnh docking. Khi đó cần kiểm tra lại file
`navigation.launch.py` đã thay và đã source workspace sau build hay chưa.

Theo dõi trạng thái:

```bash
ros2 topic echo /velocity_arbiter/state
```

Khi Nav2 đang điều khiển, trạng thái là `NAV2:ACTIVE`. Khi Nav2 đứng yên và
không còn phát lệnh mới, `NAV2:STALE` là bình thường; arbiter đang phát zero.

## 5. Kiểm tra hướng LaserScan

TF hiện tại có `yaw=-1.5708`, nên controller dùng:

```text
Hướng trước xe trong LaserScan: +1.5708 rad
Hướng sau xe trong LaserScan:   -1.5708 rad
```

Đặt một vật phẳng trước xe rồi kiểm tra:

```bash
ros2 topic echo /docking/front_distance
```

Đặt vật phía sau rồi kiểm tra:

```bash
ros2 topic echo /docking/rear_distance
```

Nếu hai hướng bị đảo trên robot thật, chỉ đổi hai tham số
`front_scan_angle` và `rear_scan_angle` trong `docking_params.yaml`; không đổi
TF chỉ để sửa controller docking.

## 6. Dock vào trạm

Đầu tiên dùng station manager đưa xe đến staging:

```bash
ros2 topic pub --once /station/command \
  std_msgs/msg/String "{data: 'station_1'}"
```

Chờ trạng thái:

```bash
ros2 topic echo /station/status
```

Phải nhận:

```text
AT_STAGING:station_1
```

Sau đó mới gửi lệnh docking:

```bash
ros2 topic pub --once /docking/command \
  std_msgs/msg/String "{data: 'dock station_1'}"
```

Theo dõi:

```bash
ros2 topic echo /docking/status
```

Kết quả cuối mong muốn:

```text
DOCKED:station_1
```

Tại `DOCKED`, arbiter chuyển sang `HOLD`, xe không tự nhận lại quyền Nav2.

## 7. Ra khỏi trạm

Kiểm tra phía sau xe thông thoáng ít nhất 0,37 m rồi gửi:

```bash
ros2 topic pub --once /docking/command \
  std_msgs/msg/String "{data: 'undock station_1'}"
```

Controller lùi từ `dock_pose` về `staging_pose` bằng lookahead + PD. Kết quả:

```text
UNDOCKED:station_1
```

Sau `UNDOCKED`, arbiter tự trả về mode `NAV2` để có thể nhận hành trình tiếp.

Thay `station_1` bằng `station_2` để thử trạm 2.

## 8. Dừng và phục hồi lỗi

Dừng docking có kiểm soát:

```bash
ros2 topic pub --once /docking/command \
  std_msgs/msg/String "{data: 'cancel'}"
```

Dừng khẩn toàn bộ vận tốc:

```bash
ros2 topic pub --once /velocity_arbiter/mode \
  std_msgs/msg/String "{data: 'FAULT'}"
```

Sau khi đã xử lý nguyên nhân, reset docking:

```bash
ros2 topic pub --once /docking/command \
  std_msgs/msg/String "{data: 'reset'}"
```

`reset` đưa arbiter về `HOLD`. Nếu muốn tiếp tục Nav2 mà không chạy UNDOCK:

```bash
ros2 topic pub --once /velocity_arbiter/mode \
  std_msgs/msg/String "{data: 'NAV2'}"
```

## 9. Các tham số cần chỉnh khi chạy thật

Các giá trị ban đầu đã được đặt chậm và bảo thủ:

```yaml
forward_speed: 0.08
reverse_speed: 0.07
lookahead_distance: 0.10
target_front_distance: 0.37
front_distance_tolerance: 0.04
final_position_tolerance: 0.08
final_yaw_tolerance_deg: 5.0
```

`target_front_distance=0.37 m` giữ nguyên khe vật lý 0,10 m vì đầu xe cách
`base_link` 0,27 m và LiDAR đặt tại `x=0` so với `base_link`.

Lần đầu chỉ thử một trạm, giữ tay ở công tắc dừng, và theo dõi đồng thời:

```bash
ros2 topic echo /docking/status
ros2 topic echo /docking/front_distance
ros2 topic echo /docking/lateral_error
ros2 topic echo /docking/heading_error
```
