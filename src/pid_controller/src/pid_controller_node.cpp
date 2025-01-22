
class pidController {
private:
  double p, i, d;
  double reference;
  double voltage;
  double integral;
  double previous_error;
  double dt;               // tidssteg

public:
  pidController(double kp, double ki, double kd, double ref);
  void update(double measured_value);
  void setReference(double new_reference);
  double get_voltage();
};


pidController::pidController(double kp, double ki, double kd, double ref)
    : p(kp), i(ki), d(kd), reference(ref), voltage(0.0),
      integral(0.0), previous_error(0.0), dt(0.1) {}

void pidController::setReference(double new_reference) {
  reference = new_reference;
}

void pidController::update(double measured_value) {
  double error = reference - measured_value;
  integral += error * dt;
  double derivative = (error - previous_error) / dt;

  voltage = p * error + i * integral + d * derivative;

  previous_error = error;
}

double pidController::get_voltage() {
  return voltage;
}



// ROS2-node
#include <crono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class PIDControllerNode : public rclcpp::Node {
private:
  void measurement_listener(const std_msgs::msg::Float64::SharedPtr msg) {
    double measured_value = msg->data;

    // oppdaterer PID med measured_value
    pidController_.update(measured_value);

    // publiserer den oppdaterte voltage
    publish_voltage();
  }

  void publish_voltage() {
    auto message = std_msgs::msg::Float64();
    message.data = pidController_.get_voltage();
    publisher_.publish(message);

    RCLCPP_INFO(this->get_logger(), "Published voltage: %f", message.data);
  }

  PIDController pidController_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_;

public:
  PIDControllerNode()
  : Node("pid_controller_node"), pid_controller_(1.0, 0.1, 0.05, 0.0) {

    // publisher for voltage
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("voltage", 10);

    // subscriber for measured_angle
    subscriber_ = this->create_subscription<std_msgs::msg::Float64>
                  ("measured_angle", 10, std::bind(&PIDControllerNode::measurement_listener, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "PID controller node started");
  }
};



// main
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDControllerNode>());
  rclcpp::shutdown();
  return 0;
}




//-----------------------------prøvde fra nettsida---------------------------------

//class PIDControllerPublisher : public rclcpp::Node {
//public:
//  PIDController() : Node("pid_controller"), count_(0) {
//    publisher_ = this->create_publisher<std_msgs::msg::String>("Voltage", 10);
//    auto timer_callback =
//    [this]() -> void {
//      auto message = std_msgs::msg::String();
//      message.data = "Voltage: f% " + std::to_string(count_++);
//      RCLCPP_INFO(this->get_logger(), "Publishing voltage");
//      this->publisher_->publish(message);
//    };
//    timer_ = this->create_wall_timer(1000ms, timer_callback);
//  }
//
//private:
//  rclcpp::TimerBase::SharedPtr timer_;
//  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
//  size_t count_;
//};