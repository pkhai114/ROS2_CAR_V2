cmake_minimum_required(VERSION 3.8)
project(robot_navigation_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    config
    maps
    rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/station_visualizer_node.py
    scripts/station_manager_node.py
    scripts/station_auto_manager_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES scripts/station_workflow.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES STATIONS_GUIDE.md STATION_AUTO_GUIDE.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(station_workflow test/test_station_workflow.py)
  ament_add_pytest_test(station_adapters test/test_station_adapters.py)
endif()

ament_package()
