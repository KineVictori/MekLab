
// pid controller
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

  void setP(double kp);
  void setI(double ki);
  void setD(double kd);
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

void pidController::setP(double kp) {
  this->p = kp;
}

void pidController::setI(double ki) {
  this->i = ki;
}

void pidController::setD(double kd) {
  this->d = kd;
}


// ROS2-node
#include <chrono>
#include <memory>
#include <string>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;

class PIDControllerNode : public rclcpp::Node {
private:
  void measurement_listener(const std_msgs::msg::Float64::SharedPtr msg) {
    double measured_value = msg->data;

    // oppdaterer PID med measured_value
    prevMeasuredAngle = measured_value; 

    // publiserer den oppdaterte voltage
    // publish_voltage();
    
    RCLCPP_INFO(this->get_logger(), "Got angle: %f", measured_value);
  }

  void updatePID() {
    pidController_.update(prevMeasuredAngle);
  }

  void publish_voltage() {
    auto message = std_msgs::msg::Float64();
    message.data = pidController_.get_voltage();
    this->publisher_->publish(message);

    RCLCPP_INFO(this->get_logger(), "Published voltage: %f", message.data);
  }

  double prevMeasuredAngle = 0.0;
  pidController pidController_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_;
  
  rclcpp::TimerBase::SharedPtr timerPublish_;
  rclcpp::TimerBase::SharedPtr timerParams_;
  rclcpp::TimerBase::SharedPtr timerUpdate_;



public:
  void readParam();

  PIDControllerNode()
  : Node("pid_controller_node"), pidController_(1.0, 0.1, 0.05, 0.0) {


    // publisher for voltage
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("Lab1KineaVoltage", 10);

    // subscriber for measured_angle
    subscriber_ = this->create_subscription<std_msgs::msg::Float64>
                  ("Lab1KineaAngle", 10, std::bind(&PIDControllerNode::measurement_listener, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "PID controller node started");
    
    timerPublish_ = this->create_wall_timer(100ms, std::bind(&PIDControllerNode::publish_voltage, this));
    
    timerUpdate_ = this->create_wall_timer(50ms, std::bind(&PIDControllerNode::updatePID, this));

    this->declare_parameter("kp", 1.0);
    this->declare_parameter("ki", 0.0);
    this->declare_parameter("kd", 0.0);
    this->declare_parameter("reference", 0.0);

    timerParams_ = this->create_wall_timer(100ms, std::bind(&PIDControllerNode::readParam, this));

    double kp = this->get_parameter("kp").as_double();
    double ki = this->get_parameter("ki").as_double();
    double kd = this->get_parameter("kd").as_double();
    double reference = this->get_parameter("reference").as_double();

    pidController_.setP(kp);
    pidController_.setI(ki);
    pidController_.setD(kd);
    pidController_.setReference(reference);
  }
};

void PIDControllerNode::readParam() {
  double myParam = this->get_parameter("kp").as_double();
  pidController_.setP(myParam);

  myParam = this->get_parameter("ki").as_double();
  pidController_.setI(myParam);

  myParam = this->get_parameter("kd").as_double();
  pidController_.setD(myParam);

  myParam = this->get_parameter("reference").as_double();
  pidController_.setReference(myParam);
}


// service node
#include "rclcpp/rclcpp.hpp"
#include <pid_controller_msgs/srv/set_reference.hpp>
#include <memory>

void add(const std::shared_ptr<pid_controller_msgs::srv::SetReference::Request> request,
         std::shared_ptr<pid_controller_msgs::srv::SetReference::Response> response) {

  request->request;
  double requested_reference = request->request;

  bool did_it_work = true;
  response->success = did_it_work;
}




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
