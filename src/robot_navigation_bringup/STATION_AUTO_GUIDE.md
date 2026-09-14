# Hệ thống trạm AUTO — bản cập nhật từ SCR_RB.zip

## Hành vi

- Gửi `station_1`: Nav2 đến staging 1; kiểm tra vị trí, hướng, xe dừng; tự dock; giữ xe tại trạm.
- Sau `DOCKED:station_1`, gửi `station_2`: tự undock trạm 1 về staging 1; kiểm tra xe dừng đúng staging; Nav2 đến staging 2; kiểm tra rồi tự dock.
- `home`: nếu đang dock thì undock trước; Nav2 đến HOME; không dock tại HOME.
- Gửi lại đúng trạm đang dock: kiểm tra lại điều kiện đỗ, không tự undock/dock lại.
- Khi đang chạy, lệnh trạm mới bị từ chối với `BUSY`; bản này không xếp hàng và không tự đổi mục tiêu giữa đường.
- Lỗi hoặc `cancel`: dừng chuỗi, giữ vận tốc 0 và chờ xử lý/reset. Không tự thử lại.

`station_auto_manager_node.py` dùng tên ROS `/station_manager_node` để giữ tương thích topic và marker cũ. File `station_manager_node.py` cũ vẫn được giữ để quay lại cách chạy thủ công, nhưng không chạy cùng manager mới.

## Cài đặt và build

Trước khi thay file: dừng nhiệm vụ, xác nhận xe đã đứng yên, chuẩn bị ngắt động lực. Dừng các launch trạm/docking cũ. Không cập nhật trong lúc xe đang di chuyển.

Ví dụ ZIP tải về tại `/home/t9/Downloads/robot_station_auto_v2.zip`. Nếu tên/thư mục tải khác, đổi đúng đường dẫn này. Hai package trong ZIP là bản đầy đủ từ SCR_RB.zip, cộng phần cập nhật; nếu bạn đã sửa thêm sau khi gửi SCR_RB.zip thì so sánh trước khi chép.

Sao lưu RA NGOÀI `src` để tránh lỗi duplicate package:

```bash
station_backup_dir=$(mktemp -d /home/t9/ros2_ws_backup_auto_XXXXXX)
cp -a /home/t9/ros2_ws/src/robot_navigation_bringup "$station_backup_dir/"
cp -a /home/t9/ros2_ws/src/robot_docking "$station_backup_dir/"
echo "$station_backup_dir"
```

Giải nén vào thư mục tạm riêng và chép đúng hai package:

```bash
station_update_dir=$(mktemp -d /home/t9/robot_station_auto_v2_XXXXXX)
unzip /home/t9/Downloads/robot_station_auto_v2.zip -d "$station_update_dir"
cp -a "$station_update_dir/robot_navigation_bringup/." /home/t9/ros2_ws/src/robot_navigation_bringup/
cp -a "$station_update_dir/robot_docking/." /home/t9/ros2_ws/src/robot_docking/

cd /home/t9/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_navigation_bringup robot_docking --symlink-install
source /home/t9/ros2_ws/install/setup.bash
```

Nếu thiếu `ament_cmake_pytest`, cài dependency kiểm thử trên máy ROS:

```bash
sudo apt install ros-humble-ament-cmake-pytest python3-pytest
```

Không đổi tên package thành một thư mục backup trong `src`. Không cần sửa hoặc build lại `robot_localization_bringup` cho thay đổi này.

## Khởi động

Giữ cách khởi động ESP32/micro-ROS Agent, IMU, LiDAR, EKF, TF, HOME/AMCL và Nav2 đang chạy ổn định của bạn. Chỉ một nguồn `map -> odom`, không chạy SLAM cùng AMCL. Không chạy thêm một Nav2/localization thứ hai.

Launch Nav2 hiện tại được giữ nguyên, bao gồm remap đã kiểm tra:

| Node | Topic vận tốc |
| --- | --- |
| controller_server | phát `/cmd_vel_nav` |
| velocity_smoother | nhận `/cmd_vel_nav`, phát `/cmd_vel_nav_output` |
| behavior_server | phát `/cmd_vel_nav_output` |
| docking_controller | phát `/cmd_vel_dock` |
| velocity_arbiter | chọn nguồn, phát duy nhất `/cmd_vel` |
| esp32_base_controller | nhận `/cmd_vel` |

Khi Nav2 và cảm biến đã sẵn sàng, mở một terminal mới:

```bash
source /opt/ros/humble/setup.bash
source /home/t9/ros2_ws/install/setup.bash
ros2 launch robot_docking stations_auto.launch.py
```

Launch này chạy đúng 4 node: arbiter, docking controller, station auto manager, station visualizer. Nó KHÔNG chạy Nav2/localization/cảm biến.

Không chạy đồng thời:

```text
ros2 launch robot_navigation_bringup stations.launch.py
ros2 launch robot_docking docking.launch.py
```

Trong AUTO, arbiter mặc định HOLD. Docking controller chỉ nhận thao tác dock/undock từ manager qua giao thức có request ID. Lệnh dock/undock thủ công qua `/docking/command` bị từ chối; `cancel` thủ công vẫn được phép dừng.

## Kiểm tra trước lần chạy đầu

Chuẩn bị nút dừng/ngắt động lực; kiểm tra đường tiến và lùi trống. Không đứng trước hoặc sau xe. Kiểm tra ESP32 có watchdog dừng động cơ khi mất `/cmd_vel`: mã ESP32 không nằm trong gói cập nhật này. Nếu arbiter/máy tính chết, phần mềm ROS không thể bảo đảm gửi thêm lệnh 0.

```bash
ros2 topic info /cmd_vel -v
ros2 node list
ros2 param get /docking_controller managed_only
ros2 param get /velocity_arbiter require_station_heartbeat
ros2 param get /docking_controller target_front_distance
ros2 param get /docking_controller front_hard_stop_distance
ros2 topic echo /velocity_arbiter/state --once
ros2 topic echo /docking/sequence_status --once
```

Kết quả cần có:

- `/cmd_vel`: chỉ 1 publisher, `velocity_arbiter`.
- Không trùng `/station_manager_node`, `/docking_controller`, `/velocity_arbiter`.
- Hai tham số `managed_only` và `require_station_heartbeat`: `True`.
- Khoảng đo đích `0.37 m`; hard stop `0.33 m`.
- Arbiter `HOLD:ZERO` khi chờ.
- JSON docking có `scan_fresh: true`; robot đã định vị đúng trên map, TF/odom cập nhật.

Mở terminal theo dõi trước khi gửi lệnh (giữ chạy):

```bash
ros2 topic echo /station/status
```

Terminal khác để xem lý do từ chối/lỗi:

```bash
ros2 topic echo /station/events
```

Trong RViz, giữ MarkerArray `/station_markers`. Hình học và nhãn trạm giữ nguyên; trạm đang xử lý màu cam, đã DOCKED màu xanh lam, thất bại/hủy màu đỏ. Không gửi `Nav2 Goal`, waypoint, teleop hoặc action chuyển động khác song song với nhiệm vụ AUTO.

## Gửi lệnh

Đến và tự dock trạm 1:

```bash
ros2 topic pub --once /station/command std_msgs/msg/String "{data: 'station_1'}"
```

Thứ tự trạng thái chính:

```text
NAVIGATING:station_1
CHECK_STAGING:station_1
WAIT_DOCK_ACK:station_1
DOCKING:station_1
CHECK_DOCK:station_1
DOCKED:station_1
```

Các trạng thái HOLD/WAIT_NAV_MODE/SENDING_GOAL có thể xuất hiện xen giữa. Sau khi xác nhận `DOCKED:station_1`, gửi:

```bash
ros2 topic pub --once /station/command std_msgs/msg/String "{data: 'station_2'}"
```

Xe thực hiện `UNDOCKING:station_1`, `CHECK_UNDOCK:station_1`, rồi `NAVIGATING:station_2`, kiểm tra staging, docking và `DOCKED:station_2`.

Về HOME:

```bash
ros2 topic pub --once /station/command std_msgs/msg/String "{data: 'home'}"
```

Chỉ gửi một lệnh bằng `--once`; không dùng publisher lặp liên tục cho lệnh trạm.

## Điều kiện chuyển bước

Trước dock phải có TẤT CẢ:

1. Kết quả action Nav2 đúng goal hiện tại là `SUCCEEDED`.
2. Sai số TF so với staging <= 0.10 m và yaw <= 5 độ.
3. Vận tốc tịnh tiến đo từ odometry <= 0.015 m/s, vận tốc góc <= 0.035 rad/s.
4. Các điều kiện trên ổn định 0.50 s; HOLD đã được arbiter xác nhận.
5. TF/scan/odom mới, heartbeat arbiter/controller còn hoạt động; các node dùng cùng nội dung stations.yaml.

Trước khi đi từ trạm cũ sang trạm mới: phải nhận UNDOCKED đúng request ID/trạm, và kiểm tra lại staging trạm cũ + hướng + dừng ổn định. Không cấp Nav2 ngay chỉ vì có chuỗi chữ `UNDOCKED`.

DOCKED cuối cùng: vị trí <= 0.08 m, yaw <= 5 độ, khoảng trước `0.37 ± 0.04 m`, vận tốc đã dừng, ổn định 0.50 s. Bộ controller vẫn căn yaw đầu/cuối và bám đường bằng lookahead + PD như bản cũ.

Các giới hạn của manager nằm trong `robot_navigation_bringup/config/station_auto_params.yaml`; giới hạn controller nằm trong `robot_docking/config/docking_params.yaml`. Sau khi sửa, build/source lại nếu cần và khởi động lại launch. Không tự nới tolerance để bỏ qua lỗi.

## Hủy, lỗi và reset

Dừng chuỗi đang chạy:

```bash
ros2 topic pub --once /station/command std_msgs/msg/String "{data: 'cancel'}"
```

Manager yêu cầu HOLD, hủy goal Nav2 của nó, hủy thao tác docking đang quản lý, chờ kết quả hủy và xe dừng. Không hủy goal của chương trình khác. `CANCELING` nghĩa là CHƯA xác nhận hủy xong, không gửi lệnh mới.

Sau `CANCELED`/`FAILED`, kiểm tra nguyên nhân, vị trí thực tế, đường đi và cảm biến. Khi đủ điều kiện mới gửi:

```bash
ros2 topic pub --once /station/command std_msgs/msg/String "{data: 'reset'}"
```

Reset chỉ xóa trạng thái lỗi sau khi thao tác cũ đã kết thúc; không di chuyển xe và không tự tiếp tục nhiệm vụ. Đợi `IDLE`, rồi gửi lại trạm mong muốn. Nếu mất liên lạc khiến không xác nhận được hủy, reset sẽ bị chặn. Dừng động lực, dừng các launch liên quan, kiểm tra goal còn tồn tại và khởi động lại có kiểm soát.

Nếu xe dừng giữa đường docking: hệ thống có thể từ chối với `RECOVERY_REQUIRED_IN_DOCKING_CORRIDOR`. Không tự đẩy goal Nav2 từ vị trí sát trạm. Cần phục hồi có giám sát về staging hoặc dock hợp lệ khi động lực được kiểm soát; kiểm tra lại localization trước khi reset/khởi động lại. Nếu nhớ đang dock nhưng pose không khớp, `DOCK_STATE_POSE_MISMATCH` cũng cần kiểm tra thủ công.

Khởi động lại khi xe ở dock: manager suy ra trạm từ pose trong vùng 0.14 m, yêu cầu yaw trong 5 độ rồi mới cho undock. Đây là suy luận từ AMCL, không phải cảm biến xác nhận vật lý. Không khởi động với HOME giả khi xe thực tế đang ở trạm khác.

## Những thay đổi và giới hạn

- Thêm state machine độc lập `station_workflow.py`, manager AUTO và launch tổng hợp.
- Thêm giao thức JSON nội bộ có UUID thao tác, ACK, heartbeat, chống lặp và bỏ kết quả cũ. Các topic public `/station/command`, `/station/status`, `/docking/status` vẫn là String.
- Managed undock giữ HOLD, chờ manager kiểm tra lại; manual undock với launch cũ vẫn trả NAV2.
- Arbiter trong AUTO dừng khi mất heartbeat manager hoặc phát hiện trùng manager. Timeout nguồn vận tốc vẫn 0.30 s; manager heartbeat timeout 1.0 s. Đây là watchdog phần mềm, không phải E-stop thời gian thực.
- Không thay tọa độ, kích thước/định hướng marker, map, Nav2 tolerance, remap vận tốc, HOME hay bộ lookahead/PD.
- Có thay `front_hard_stop_distance` từ 0.27 thành 0.33 m để nằm trên ngưỡng lọc scan 0.30 m đã được nêu trong cấu hình dự án. Không đổi `target_front_distance=0.37` hoặc khe vật lý danh định 0.10 m. Xác minh bộ lọc LiDAR thực tế trước thử nghiệm.
- Kiểm tra scan hiện hữu chỉ xét sector trước/sau ±10 độ; KHÔNG bảo vệ toàn bộ footprint/góc xe khi quay, không thay thế bumper, E-stop hoặc collision monitor. Vùng scan bị lọc/không nhìn thấy không thể được xác nhận là trống.
- Không có persistent mission/resume tự động, không queue, không preempt sang trạm khác khi busy, không chạy song song với teleop/goal ngoài manager.

## Kiểm thử

Chạy offline, không cần ROS và không phát lệnh tới robot:

```bash
cd /home/t9/ros2_ws/src
python3 -m unittest discover -s robot_navigation_bringup/test -v
```

Hoặc sau build:

```bash
cd /home/t9/ros2_ws
colcon test --packages-select robot_navigation_bringup
colcon test-result --verbose
```

Các test bao gồm trình tự station 1 -> station 2, HOME, gate pose/yaw/dừng/khoảng cách, kết quả cũ/sai ID/sai trạm, timeout, hủy trước khi Nav2 nhận goal, kết quả tức thời, chống duplicate và watchdog. Test adapter dùng ROS message/action giả để gọi các phương thức nguồn thật; không phải mô phỏng động học hoặc thử trên ROS graph thật.

Môi trường tạo gói không có ROS 2 Humble và không kết nối xe: chưa chạy colcon build, DDS, AMCL/Nav2 hay đo quãng đường phanh thực tế ở đây. Chỉ thử tự động có giám sát, từng trạm trước, rồi mới thử chuyển trạm.

Tham khảo API: [ROS 2 Humble action client](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html) cho luồng gửi goal, xác nhận và kết quả bất đồng bộ; [ROS 2 actions](https://docs.ros.org/en/humble/Concepts/Basic/About-Actions.html) cho ngữ nghĩa nhiệm vụ có kết quả/hủy.
