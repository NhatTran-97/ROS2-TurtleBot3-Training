cmake_minimum_required(VERSION 3.8)
project(simple_cpp_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)   
find_package(image_transport REQUIRED) 
find_package(simple_velocity_msg REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(simple_publisher src/simple_publisher.cpp)
ament_target_dependencies(simple_publisher rclcpp std_msgs)

add_executable(simple_subscriber src/simple_subscriber.cpp)
ament_target_dependencies(simple_subscriber rclcpp std_msgs)

add_executable(move_robot src/move_robot.cpp)
ament_target_dependencies(move_robot rclcpp nav_msgs geometry_msgs sensor_msgs)

add_executable(image_subscriber_node src/image_subscriber.cpp)
ament_target_dependencies(image_subscriber_node rclcpp sensor_msgs cv_bridge OpenCV)



add_executable(custom_send_twist_msg_node src/custom_send_twist_msg.cpp)
ament_target_dependencies(custom_send_twist_msg_node rclcpp sensor_msgs simple_velocity_msg geometry_msgs)

add_executable(custom_control_robot_node src/custom_control_robot.cpp)
ament_target_dependencies(custom_control_robot_node rclcpp sensor_msgs simple_velocity_msg geometry_msgs)


install(TARGETS 
    simple_publisher
    simple_subscriber
    move_robot
    image_subscriber_node
    custom_send_twist_msg_node
    custom_control_robot_node
DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
