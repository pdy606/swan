cmake_minimum_required(VERSION 3.8)
project(swan_world_integration)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch models worlds config
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/swan_drive_controller.py
  scripts/animate_crosswalk.py
  scripts/start_recording.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
