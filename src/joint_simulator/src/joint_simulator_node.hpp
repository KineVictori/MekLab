
#ifndef MEKLAB_JOINT_SIMULATOR_NODE_HPP
#define MEKLAB_JOINT_SIMULATOR_NODE_HPP

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class jointSimulator {
private:

    double angle;
    double angular_velocity;
    double voltage;

    double K;
    double T;
    double noise;

    std::chrono::time_point<std::chrono::high_resolution_clock> prevTime;
public:

    jointSimulator();
    void update();
    double get_angle();
    void set_voltage(double voltage);

    void set_K(double K);
    void set_T(double T);
    void set_noise(double noise);
    
    double get_K();
};

class jointSimulatorNode : public rclcpp::Node {
public:

    jointSimulatorNode();
	void update();

	void publishMessage(double messageData);
	void readMessage(std_msgs::msg::Float64::UniquePtr msg);

private:

    jointSimulator simulator;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;

	rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter> &params);
};

#endif //MEKLAB_JOINT_SIMULATOR_NODE_HPP

