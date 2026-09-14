cmake_minimum_required(VERSION 3.8)
project(robot_docking)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/velocity_arbiter_node.py
    scripts/docking_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES DOCKING_GUIDE.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
