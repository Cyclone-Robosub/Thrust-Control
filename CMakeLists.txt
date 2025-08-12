cmake_minimum_required(VERSION 3.8)
project(thrust_control)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

add_subdirectory(include/exported-PID-control)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(crs_ros2_interfaces REQUIRED)

# Optional mocking flag
option(MOCK_RPI "Use mock Pico" OFF)
option(BUILD_TESTING "Build Gtest executables" OFF)

if (MOCK_RPI)
    add_compile_options("-DMOCK_RPI")
endif()

add_subdirectory(include/Command_Interpreter)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()


add_executable(thrust_control 
  src/main.cpp
  src/thrust_control_node.cpp
  src/thrust_control_node.hpp
  src/thrust_control_supervisor.cpp
  src/thrust_control_supervisor.hpp
  src/command_interpreter_pointer.cpp
  src/command_interpreter_pointer.hpp
  src/command_queue.cpp
  src/command_queue.hpp
  src/supervisor_command.cpp
  src/supervisor_command.hpp
  src/position.cpp
  src/position.hpp
)

target_include_directories(thrust_control
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Command_Interpreter/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include/exported-PID-control
    ${CMAKE_CURRENT_SOURCE_DIR}/include/exported-PID-control/src
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
ament_target_dependencies(thrust_control rclcpp std_msgs crs_ros2_interfaces)
target_link_libraries(thrust_control
    PropulsionFunctions
    controller_lib
)

install(TARGETS
  thrust_control
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  #ament_lint_auto_find_test_dependencies()
  
  find_package(GTest REQUIRED)
  enable_testing()

  add_executable(thrust_control_tests
    tests/test_thrust_control_node.cpp
    tests/test_thrust_control_supervisor.cpp
    tests/test_command_queue.cpp
    tests/test_command.cpp
    src/thrust_control_node.cpp
    src/thrust_control_node.hpp
    src/thrust_control_supervisor.cpp
    src/thrust_control_supervisor.hpp
    src/command_interpreter_pointer.cpp
    src/command_interpreter_pointer.hpp
    src/command_queue.cpp
    src/command_queue.hpp
    src/supervisor_command.cpp
    src/supervisor_command.hpp
    src/position.cpp
    src/position.hpp
    src/pwm_message_utils.cpp
    src/pwm_message_utils.hpp
  )

  target_include_directories(thrust_control_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Command_Interpreter/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include/exported-PID-control
    ${CMAKE_CURRENT_SOURCE_DIR}/include/exported-PID-control/src
    ${GTEST_INCLUDE_DIRS}
  )

  ament_target_dependencies(thrust_control_tests rclcpp std_msgs crs_ros2_interfaces)

  target_link_libraries(thrust_control_tests
    PropulsionFunctions
    controller_lib
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
    GTest::gtest_main
  )
  # Register the test
  gtest_discover_tests(thrust_control_tests)
endif()

ament_export_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
ament_export_dependencies(rclcpp std_msgs crs_ros2_interfaces)

ament_package()
