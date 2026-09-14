cmake_minimum_required(VERSION 3.8)
project(robot_localization_bringup)

find_package(ament_cmake REQUIRED)

# Cài đặt file config và launch.
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Cài đặt Python node thành ROS2 executable.
install(
  PROGRAMS
    scripts/home_initial_pose_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()