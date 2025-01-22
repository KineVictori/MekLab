
#include "joint_simulator_node.hpp"

jointSimulator::jointSimulator(double angle, double angular_velocity, double noise, double voltage) {
  this->angle = angle;
  this->angular_velocity = angular_velocity;
  this->noise = noise;
  this->voltage = voltage;

  this->prevTime = std::chrono::high_resolution_clock::now();
}

void jointSimulator::update() {
    double K = 230.0;
    double T = 0.15;

    auto currTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = currTime - prevTime;
    prevTime = currTime;

    auto dt = duration.count();

    angle += dt * angular_velocity;
    angular_velocity += dt * (-angular_velocity / T - K * voltage / T);
}

double jointSimulator::get_angle() {
    return angle;
}

void jointSimulator::set_voltage(double voltage) {
    this->voltage = voltage;
}


jointSimulatorNode::jointSimulatorNode(): Node("Angle_Publisher"), simulator(0.0, 0.0, 0.0, 1.0) {

    publisher_ = this->create_publisher<std_msgs::msg::String>("Lab1Kinea", 10);
    auto timer_callback =
            [this]() -> void {

                simulator.update();

                auto message = std_msgs::msg::String();
                message.data = std::to_string(simulator.get_angle());
                RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
                this->publisher_->publish(message);
            };
    timer_ = this->create_wall_timer(100ms, timer_callback);
}


