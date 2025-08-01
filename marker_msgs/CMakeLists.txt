cmake_minimum_required(VERSION 3.8)
project(marker_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(srv_files
  "srv/FindAstarPath.srv"
  "srv/Navigate.srv"
  )

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MarkerPose.msg"
  "msg/MarkerPoseArray.msg"
  "msg/MarkerPoint.msg"
  "msg/MarkerPointArray.msg"
  "action/NavigateToMarker.action"
  ${srv_files} 
  DEPENDENCIES std_msgs geometry_msgs action_msgs
)


ament_export_dependencies(rosidl_default_runtime)

ament_package()
