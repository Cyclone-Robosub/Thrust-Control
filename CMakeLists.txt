cmake_minimum_required(VERSION 3.8)
project(thrust_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(include)

# Add thrust control node executable
add_executable(thrust_control src/thrust_control_node.cpp)
ament_target_dependencies(thrust_control rclcpp std_msgs)

# In the install targets section, add:
install(TARGETS
  thrust_control
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export package dependencies
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs)

ament_package()
