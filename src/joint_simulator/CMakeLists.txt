cmake_minimum_required(VERSION 3.8)
project(joint_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/main.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

install(TARGETS
        talker
        DESTINATION lib/${joint_simulator})

ament_package()
