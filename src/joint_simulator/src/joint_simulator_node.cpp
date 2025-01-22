
#include <functional>
#include "joint_simulator_node.hpp"

jointSimulator::jointSimulator() {
	this->angle = 0.0;
	this->angular_velocity = 0.0;
	this->voltage = 0.0;

	this->K = 200.0;
	this->T = 0.15;
    this->noise = 0.0;

  this->prevTime = std::chrono::high_resolution_clock::now();
}

void jointSimulator::update() {

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

double jointSimulator::get_K() {
        return this->K;
}

void jointSimulatorNode::publishMessage(double messageValue) {
	auto message = std_msgs::msg::Float64();
    message.data = messageValue;
	this->publisher_->publish(message);

	// Publishes debug/message to the console.
	RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.data);
    // Kan også bruke "ros2 topic echo /angle"
	// Aka: ros2 topic echo /Lab1KineaAngle
}

void jointSimulatorNode::update() {

	simulator.update();
	publishMessage(simulator.get_angle());
}

void jointSimulatorNode::readMessage(std_msgs::msg::Float64::UniquePtr msg) {
	RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->data);

	simulator.set_voltage(msg->data);
}

void jointSimulatorNode::readParam() {
	double myParam = this->get_parameter("K").as_double();
	simulator.set_K(myParam);

	myParam = this->get_parameter("T").as_double();
	simulator.set_T(myParam);

	myParam = this->get_parameter("noise").as_double();
	simulator.set_noise(myParam);

}

jointSimulatorNode::jointSimulatorNode(): Node("Joint_Simulator"), simulator() {

    publisher_ = this->create_publisher<std_msgs::msg::Float64>("Lab1KineaAngle", 10);

    timerPublish_ = this->create_wall_timer(100ms, std::bind(&jointSimulatorNode::update, this));

	subscription_ = this->create_subscription<std_msgs::msg::Float64>("Lab1KineaVoltage", 10, std::bind(&jointSimulatorNode::readMessage, this, std::placeholders::_1));

	this->declare_parameter("noise", 0.0);
	this->declare_parameter("K", 230.0);
	this->declare_parameter("T", 0.15);

	timerParam_ = this->create_wall_timer(100ms, std::bind(&jointSimulatorNode::readParam, this));
	// auto successReturn = this->add_on_set_parameters_callback(std::bind(&jointSimulatorNode::parameter_callback, this, std::placeholders::_1));
}

//rcl_interfaces::msg::SetParametersResult jointSimulatorNode::parameter_callback(const std::vector<rclcpp::Parameter> &params)
//{
//
//	rcl_interfaces::msg::SetParametersResult result;
//	result.successful = false;
//
//	// this->get_parameter("K", this->simulator.K);
//
//	for (const auto &param : params)
//	{
//		if (param.get_name() == "noise")
//		{
//			if (param.as_double() >= 0.0)
//			{
//				double noise = param.as_double();
//				this->simulator.set_noise(noise);
//				RCLCPP_INFO(this->get_logger(), "Noise parameter updated: %f", noise);
//			} else {
//				RCLCPP_INFO(this->get_logger(), "Noise parameter must be more than zero!");
//			}
//		}
//		else if (param.get_name() == "K")
//		{
//			double K = param.as_double();
//			this->simulator.set_K(K);
//			RCLCPP_INFO(this->get_logger(), "K parameter updated: %f", K);
//
//			result.successful = true;
//		}
//		else if (param.get_name() == "T")
//		{
//			double T = param.as_double();
//			this->simulator.set_T(T);
//			RCLCPP_INFO(this->get_logger(), "T parameter updated: %f", T);
//		}
//	}
//
//	return result;
//}
