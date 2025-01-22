
cmake_minimum_required(VERSION 3.8)
project(joint_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add the executable and include all required source files
add_executable(RunJointSim
        src/main.cpp
        src/joint_simulator_node.cpp
)

# Link dependencies
ament_target_dependencies(RunJointSim
        rclcpp
        std_msgs
)

# Install the executable
install(TARGETS
        RunJointSim
        DESTINATION lib/${PROJECT_NAME}
)

# Mark this package as an ament package
ament_package()
