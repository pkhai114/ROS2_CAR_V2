# robot_localization_bringup

Package định vị cho Robot Arm Car trên ROS 2 Humble:

- EKF hợp nhất wheel odometry và IMU.
- Map Server nạp bản đồ đã lưu.
- AMCL xác định vị trí xe trên bản đồ bằng `/scan/filtered`.

## Nguồn dữ liệu

- `/wheel/odom`: chỉ hợp nhất `twist.linear.x` (`vx`).
- `/imu/data`: hợp nhất orientation `yaw` và `angular_velocity.z` (`wz`).

## Đầu ra

- Topic `/odometry/filtered`.
- TF động `odom -> base_link`.
- Static TF `base_link -> imu_link`, với `xyz = [0, 0, 0.11]` và
  `rpy = [0, 0, 0]`.

ESP32 không được phát TF `odom -> base_link`. Không chạy launch IMU thử nghiệm
có node phát TF `map -> imu_link` cùng lúc với package này.

## Build

Chép thư mục `robot_localization_bringup` vào `~/ros2_ws/src`, sau đó:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select robot_localization_bringup \
  --symlink-install

source install/setup.bash
```

## Chạy

### 1. Chạy EKF

Trước tiên chạy micro-ROS Agent/ESP32 và driver IMU bình thường. Sau đó:

```bash
ros2 launch robot_localization_bringup ekf.launch.py
```

### 2. Chạy Map Server + AMCL

Dừng SLAM Toolbox trước, nhưng giữ ESP32, IMU, Lidar/filter và EKF đang chạy.

Với bản đồ mặc định:

```bash
ros2 launch robot_localization_bringup localization.launch.py
```

Đường dẫn mặc định:

```text
~/ros2_ws/maps/robot_map.yaml
```

Để dùng bản đồ khác:

```bash
ros2 launch robot_localization_bringup localization.launch.py \
  map:=/home/t9/ros2_ws/maps/robot_map_02.yaml
```

Trong RViz2:

1. Đặt `Fixed Frame = map`.
2. Add `Map`, topic `/map`.
3. Add `LaserScan`, topic `/scan/filtered`.
4. Add `PoseWithCovariance`, topic `/amcl_pose`.
5. Chọn `2D Pose Estimate`, nhấp đúng vị trí xe và kéo mũi tên theo
   hướng đầu xe.

## Kiểm tra EKF

```bash
ros2 topic hz /odometry/filtered
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link imu_link
```

Kiểm tra đúng biến đầu vào mà EKF đang sử dụng:

```bash
ros2 param get /ekf_filter_node odom0_config
ros2 param get /ekf_filter_node imu0_config
```

Kết quả `odom0_config` phải chỉ có `vx` là `true`. Kết quả `imu0_config` phải
chỉ có `yaw` và `vyaw` là `true`.

## Kiểm tra chuyển động

- Đẩy xe thẳng về trước: `pose.pose.position.x` phải tăng.
- Xoay xe ngược chiều kim đồng hồ: yaw phải tăng và `twist.twist.angular.z`
  phải dương.
- Để xe đứng yên: vị trí và yaw không được trôi nhanh bất thường.

## Kiểm tra AMCL

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map odom
ros2 topic info /scan/filtered
```

Kết quả đạt:

- `/map_server` và `/amcl` đều ở trạng thái `active [3]`.
- `/scan/filtered` chỉ có một publisher.
- Có TF liên tục `map -> odom`.
- Đám mây hạt AMCL hội tụ quanh xe sau khi đặt `2D Pose Estimate`.
- Khi lái xe chậm rồi quay về vị trí cũ, hình xe và LaserScan vẫn khớp với
  tường trên bản đồ.

Không chạy SLAM Toolbox đồng thời với AMCL vì cả hai đều có thể phát TF
`map -> odom`.
