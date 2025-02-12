cmake_minimum_required(VERSION 3.8)
project(pid_controller_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetReference.srv"
)

find_package(pid_controller_msgs REQUIRED)

add_executable(client src/reference_input_node.cpp)
ament_target_dependencies(client rclcpp pid_controller_msgs)

install(TARGETS
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
