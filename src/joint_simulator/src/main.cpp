
#include <rclcpp/rclcpp.hpp>

#include "joint_simulator_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<jointSimulatorNode>());
    rclcpp::shutdown();
    return 0;
}