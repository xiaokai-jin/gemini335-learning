cmake_minimum_required(VERSION 3.8)
project(gemini335_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    ../../python/object_detection_color.py
    ../../python/object_detection_aruco.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
