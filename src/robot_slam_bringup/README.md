# robot_slam_bringup

Package mapping 2D cho Robot Arm Car trên ROS 2 Humble.

## Cây TF và quyền sở hữu

```text
map                       SLAM Toolbox phát TF động
└── odom
    └── base_link         EKF phát TF động
        ├── imu_link      static TF trong robot_localization_bringup
        └── laser         static TF trong package này
```

Không chạy thêm bất kỳ node nào khác phát `map -> odom` hoặc
`odom -> base_link`.

Static TF laser được cấu hình theo kết quả kiểm tra thực tế:

- Translation: `[0.0, 0.0, 0.22]` m.
- Rotation: `[0.0, 0.0, -pi/2]` rad.

## Dữ liệu SLAM

- Scan đầu vào: `/scan/filtered`.
- Scan frame: `laser`.
- Base frame: `base_link`.
- Odom frame: `odom`.
- Map frame: `map`.
- SLAM Toolbox xuất `/map` và TF `map -> odom`.

## Chạy từng tầng

Nếu IMU, Lidar/filter và EKF đã chạy, chỉ khởi động SLAM:

```bash
ros2 launch robot_slam_bringup slam.launch.py
```

## Chạy toàn bộ mapping

Sau khi micro-ROS Agent và ESP32 đã hoạt động:

```bash
ros2 launch robot_slam_bringup mapping.launch.py
```

Launch này chạy IMU, Lidar/filter, EKF, static TF laser và SLAM Toolbox.

## Kiểm tra

```bash
ros2 lifecycle get /slam_toolbox
ros2 topic hz /scan/filtered
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo base_link laser
```

Trong RViz2, đặt `Fixed Frame` thành `map`, sau đó thêm `Map`, `LaserScan`
và `TF`.

## Lưu bản đồ

Tạo thư mục trước:

```bash
mkdir -p ~/maps
```

Lưu ảnh occupancy map và YAML:

```bash
ros2 service call /slam_toolbox/save_map \
  slam_toolbox_msgs/srv/SaveMap \
  "{name: {data: '/home/t9/maps/robot_map'}}"
```

Lưu thêm pose graph để có thể tiếp tục mapping sau này:

```bash
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox_msgs/srv/SerializePoseGraph \
  "{filename: '/home/t9/maps/robot_map'}"
```
