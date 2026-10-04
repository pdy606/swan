cmake_minimum_required(VERSION 3.8)
project(swan_kit)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    models
    worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/swan_kit_drive_controller.py
    scripts/swan_kit_avoidance_node.py
    scripts/swan_kit_situation_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
