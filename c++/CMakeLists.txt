cmake_minimum_required(VERSION 3.16)
project(orbbec_rgb_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(ORBBEC_ROOT "/home/xiaokai/orbbec_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK")
set(ORBBEC_INCLUDE_DIR "${ORBBEC_ROOT}/include")
set(ORBBEC_LIBRARY_DIR "${ORBBEC_ROOT}/lib/x64")

find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc imgcodecs)

add_executable(orbbec_rgb_demo main.cpp)
target_include_directories(orbbec_rgb_demo PRIVATE "${ORBBEC_INCLUDE_DIR}")
target_link_directories(orbbec_rgb_demo PRIVATE "${ORBBEC_LIBRARY_DIR}")
target_link_libraries(orbbec_rgb_demo PRIVATE OrbbecSDK ${OpenCV_LIBS})
set_target_properties(orbbec_rgb_demo PROPERTIES
    BUILD_RPATH "${ORBBEC_LIBRARY_DIR}"
    INSTALL_RPATH "${ORBBEC_LIBRARY_DIR}"
)
