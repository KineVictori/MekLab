
#include "joint_simulator_node.hpp"

jointSimulator::jointSimulator() {
	this->angle = 0.0;
	this->angular_velocity = 0.0;
	this->voltage = 0.0;

	this->K = 230.0;
	this->T = 0.15;
    this->noise = 0.0;

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

void jointSimulator::set_T(double T) {
    this->T = T;
}

void jointSimulator::set_K(double K) {
	this->K = K;
}

void jointSimulator::set_noise(double noise) {
	this->noise = noise;
}

jointSimulatorNode::jointSimulatorNode(): Node("Joint_Simulator"), simulator() {

    publisher_ = this->create_publisher<std_msgs::msg::Float64>("Lab1KineaAngle", 10);
    auto timer_callback =
            [this]() -> void {

                simulator.update();

                auto message = std_msgs::msg::Float64();
                message.data = simulator.get_angle();

				// Publishes debug/message to the console.
				RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.data);
                // Kan også bruke "ros2 topic echo /angle"
				// Aka: ros2 topic echo /Lab1KineaAngle

				this->publisher_->publish(message);
            };

    timer_ = this->create_wall_timer(100ms, timer_callback);

	auto topic_callback =
		[this](std_msgs::msg::Float64::UniquePtr msg) -> void {
	  		// Published debug/message to the console.
			RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->data);

			simulator.set_voltage(msg->data);
		};
	subscription_ = this->create_subscription<std_msgs::msg::Float64>("Lab1KineaVoltage", 10, topic_callback);
}


