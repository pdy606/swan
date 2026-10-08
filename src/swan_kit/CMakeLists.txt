cmake_minimum_required(VERSION 3.8)
project(swan_kit)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    models
    worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/swan_kit_drive_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
