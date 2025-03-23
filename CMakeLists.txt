cmake_minimum_required(VERSION 3.8)
project(jetank_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Test-only dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake REQUIRED)
  find_package(control_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
endif()

# Install directories
install(DIRECTORY
  launch
  config
  urdf
  meshes
  worlds
  scripts
  DESTINATION share/${PROJECT_NAME}/
)

# Test settings
if(BUILD_TESTING)
  # Skip copyright and cpplint checks unless needed
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
