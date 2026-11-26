cmake_minimum_required(VERSION 3.8)
project(gemini335_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    ../../python/object_detection_color.py
    ../../python/object_detection_aruco.py
    ../../python/object_detection_yolo.py
  DESTINATION lib/${PROJECT_NAME}
)

set(YOLOE_MODEL "${CMAKE_CURRENT_SOURCE_DIR}/../../models/yoloe-26s-seg.pt")
if(EXISTS "${YOLOE_MODEL}")
  install(
    FILES "${YOLOE_MODEL}"
    DESTINATION share/${PROJECT_NAME}/models
  )
endif()

ament_package()
