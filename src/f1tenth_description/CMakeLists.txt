cmake_minimum_required(VERSION 3.8)
project(f1tenth_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

install(DIRECTORY
  launch
  urdf
  config
  maps
  DESTINATION share/${PROJECT_NAME}
)
add_executable(steering_joint_state_publisher src/steering_joint_state_publisher.cpp)
ament_target_dependencies(steering_joint_state_publisher rclcpp std_msgs sensor_msgs)
install(TARGETS steering_joint_state_publisher
  DESTINATION lib/${PROJECT_NAME}
)   
add_executable(control_bridge src/control_bridge.cpp)
ament_target_dependencies(control_bridge rclcpp geometry_msgs ackermann_msgs)
install(TARGETS control_bridge DESTINATION lib/${PROJECT_NAME})
add_executable(imu_bridge src/imu_bridge.cpp)
ament_target_dependencies(imu_bridge rclcpp sensor_msgs)
install(TARGETS imu_bridge DESTINATION lib/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
