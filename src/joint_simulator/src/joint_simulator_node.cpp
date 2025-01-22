
#include "joint_simulator_node.hpp"

jointSimulator::jointSimulator(double angle, double angular_velocity, double noise, double voltage) {
  this->angle = angle;
  this->angular_velocity = angular_velocity;
  this->noise = noise;
  this->voltage = voltage;
}

void jointSimulator::update() {
    double K = 230.0;
    double T = 0.15;

    auto currTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = currTime - prevTime;
    prevTime = currTime;

    auto dt = duration.count();

    angle += dt * angular_velocity;
    angular_velocity += dt * (-angular_velocity * K * voltage / T);
}

void jointSimulator::get_angle() {
    return angle;
}

void jointSimulator::set_voltage(double voltage) {
    this->voltage = voltage;
}


jointSimulatorNode::jointSimulatorNode(): Node("Angle_Publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("Lab1Kinea", 10);
    auto timer_callback =
            [this]() -> void {
                auto message = std_msgs::msg::String();
                message.data = "Hello, world! " + std::to_string(this->count_++);
                RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
                this->publisher_->publish(message);
            };
    timer_ = this->create_wall_timer(500ms, timer_callback);
}


