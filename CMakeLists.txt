cmake_minimum_required(VERSION 3.8)
project(thrust_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_subdirectory(include/Command_Interpreter)
add_subdirectory(include/exported-PID-control)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic  -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

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
ament_target_dependencies(thrust_control rclcpp std_msgs)
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
  ament_lint_auto_find_test_dependencies()
  
  find_package(GTest REQUIRED)
  enable_testing()
  
  # command queue testing
  
   add_executable(test_command_queue
      tests/test_command_queue.cpp
      src/command_queue.cpp
      src/supervisor_command.cpp
  )
 
  target_include_directories(test_command_queue PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CMAKE_CURRENT_SOURCE_DIR}/include/Command_Interpreter/src
      ${GTEST_INCLUDE_DIRS}
  )

   target_link_libraries(test_command_queue
    controller_lib
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
  ) 


  # node testing
  add_executable(test_thrust_control_node
      tests/test_thrust_control_node.cpp
      src/command_interpreter_pointer.cpp
      src/thrust_control_supervisor.cpp
      src/thrust_control_node.cpp
      src/command_queue.cpp
      src/supervisor_command.cpp
  )
 
  target_include_directories(test_thrust_control_node PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CMAKE_CURRENT_SOURCE_DIR}/include/Command_Interpreter/src
      ${CMAKE_CURRENT_SOURCE_DIR}/include/exported-PID-control
      ${CMAKE_CURRENT_SOURCE_DIR}/include/exported-PID-control/src
      ${GTEST_INCLUDE_DIRS}
  )

  ament_target_dependencies(test_thrust_control_node  rclcpp std_msgs)
   target_link_libraries(test_thrust_control_node
    PropulsionFunctions
    controller_lib
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
  ) 

  # supervisor testing
  add_executable(test_thrust_control_supervisor 
    tests/test_thrust_control_supervisor.cpp
    src/command_interpreter_pointer.cpp
    src/thrust_control_supervisor.cpp
    src/command_queue.cpp
    src/supervisor_command.cpp
  )

  target_include_directories(test_thrust_control_supervisor PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CMAKE_CURRENT_SOURCE_DIR}/include/Command_Interpreter/src
      ${CMAKE_CURRENT_SOURCE_DIR}/include/exported-PID-control
      ${CMAKE_CURRENT_SOURCE_DIR}/include/exported-PID-control/src
      ${GTEST_INCLUDE_DIRS}
  )
  
  ament_target_dependencies(test_thrust_control_supervisor rclcpp std_msgs)
  target_link_libraries(test_thrust_control_supervisor
    PropulsionFunctions
    controller_lib
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
  )

  target_link_libraries(test_command_queue
    PropulsionFunctions
    controller_lib
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
  )


  target_link_libraries(test_thrust_control_supervisor
    PropulsionFunctions
    controller_lib
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
  )

  
  # Register the test
  add_test(NAME test_thrust_control_supervisor COMMAND test_thrust_control_supervisor)
endif()

ament_export_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
ament_export_dependencies(rclcpp std_msgs)

ament_package()
