cmake_minimum_required(VERSION 3.8)
project(pid_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pid_controller_msgs REQUIRED)

include_directories(launch)

add_executable(runPID src/pid_controller_node.cpp)
ament_target_dependencies(runPID rclcpp std_msgs pid_controller_msgs)

add_executable(client src/reference_input_node.cpp)
ament_target_dependencies(client rclcpp pid_controller_msgs)

install(TARGETS
  client
  runPID
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
