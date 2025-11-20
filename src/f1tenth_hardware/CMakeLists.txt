cmake_minimum_required(VERSION 3.16)
project(f1tenth_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)

# Create the hardware interface library
add_library(
  f1tenth_hardware
  SHARED
  src/f1tenth_hardware_interface.cpp
)

target_compile_features(f1tenth_hardware PUBLIC cxx_std_17)
target_include_directories(f1tenth_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/f1tenth_hardware>
)

ament_target_dependencies(
  f1tenth_hardware PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  ackermann_msgs
  vesc_msgs
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(f1tenth_hardware PRIVATE "F1TENTH_HARDWARE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface f1tenth_hardware.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/f1tenth_hardware
)

install(TARGETS f1tenth_hardware
  EXPORT export_f1tenth_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES f1tenth_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(test_f1tenth_hardware
    test/test_f1tenth_hardware_interface.cpp
  )
  target_link_libraries(test_f1tenth_hardware f1tenth_hardware)
  ament_target_dependencies(
    test_f1tenth_hardware
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )
endif()

ament_export_targets(export_f1tenth_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  ackermann_msgs
  vesc_msgs
)
ament_package()