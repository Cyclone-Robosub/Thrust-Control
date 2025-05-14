cmake_minimum_required(VERSION 3.8)
project(thrust_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include current directory for headers (option 2)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Add thrust control node executable
add_executable(thrust_control 
  thrust_control_node.cpp
  thrust_control_supervisor.cpp
)
ament_target_dependencies(thrust_control rclcpp std_msgs)

# Install targets
install(TARGETS
  thrust_control
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export package dependencies
ament_export_include_directories(.)
ament_export_dependencies(rclcpp std_msgs)

ament_package()
