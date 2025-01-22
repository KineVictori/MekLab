
#ifndef MEKLAB_JOINT_SIMULATOR_NODE_HPP
#define MEKLAB_JOINT_SIMULATOR_NODE_HPP

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class jointSimulator {
private:

    double angle;
    double angular_velocity;
    double noise;
    double voltage;

    std::chrono::time_point<std::chrono::high_resolution_clock> prevTime;
public:

    jointSimulator(double angle, double angular_velocity, double noise, double voltage);
    void update();
    double get_angle();
    void set_voltage(double voltage);
};

class jointSimulatorNode : public rclcpp::Node {
public:

    jointSimulatorNode();

private:

    jointSimulator simulator;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

#endif //MEKLAB_JOINT_SIMULATOR_NODE_HPP

