cmake_minimum_required(VERSION 3.8)
project(rb300_nav)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/slam_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
