# robot_localization_bringup

Package EKF cho Robot Arm Car trên ROS 2 Humble.

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

Trước tiên chạy micro-ROS Agent/ESP32 và driver IMU bình thường. Sau đó:

```bash
ros2 launch robot_localization_bringup ekf.launch.py
```

## Kiểm tra

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
