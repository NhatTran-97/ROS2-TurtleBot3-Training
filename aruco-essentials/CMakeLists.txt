cmake_minimum_required(VERSION 3.8)
project(aruco_essentials)

# --- ROS 2 and dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(realsense2 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(marker_msgs REQUIRED)

# --- C++ Executable ---
add_executable(logi_aruco_node src/logitech_node.cpp)

ament_target_dependencies(logi_aruco_node
  rclcpp
  geometry_msgs
  visualization_msgs
  OpenCV
  tf2
  tf2_geometry_msgs
)

target_link_libraries(logi_aruco_node
  ${geometry_msgs_TARGETS}
  ${marker_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

install(TARGETS
  # realsense_aruco_node
  logi_aruco_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
