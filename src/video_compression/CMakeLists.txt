cmake_minimum_required(VERSION 3.5)
project(video_compression)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${ZMQ_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
  ${GSTREAMER_APP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

# Create the executable
add_executable(compress src/compress_node.cpp)
add_executable(decompress src/decompress_node.cpp)
add_executable(raw src/raw_node.cpp)

ament_target_dependencies(compress rclcpp sensor_msgs)
ament_target_dependencies(decompress rclcpp sensor_msgs cv_bridge)
ament_target_dependencies(raw rclcpp sensor_msgs)

target_link_libraries(compress
  ${ZMQ_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER_APP_LIBRARIES}
)
target_link_libraries(decompress
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER_APP_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)
target_link_libraries(raw
  ${ZMQ_LIBRARIES}
  ${ament_LIBRARIES}
  yaml-cpp
)

# Install the executable
install(TARGETS
  compress
  decompress
  raw
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  video_compression/replay.py
  video_compression/capture.py
  DESTINATION lib/${PROJECT_NAME}
)

# Optionally also install the setup.py file itself and resource index
install(PROGRAMS
  resource/${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}
)

# === Optional: install launch files ===
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
