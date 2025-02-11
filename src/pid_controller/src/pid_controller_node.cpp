// ROS2-node
#include <chrono>
#include <memory>
#include <string>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;

// service node
#include "rclcpp/rclcpp.hpp"
#include <pid_controller_msgs/srv/set_reference.hpp>
#include <memory>
#include <cmath>


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



// ROS2 node

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
  rclcpp::Service<pid_controller_msgs::srv::SetReference>::SharedPtr service_;
  
  rclcpp::TimerBase::SharedPtr timerPublish_;
  rclcpp::TimerBase::SharedPtr timerParams_;
  rclcpp::TimerBase::SharedPtr timerUpdate_;


  // service node
  void handle_request_reference(const
         std::shared_ptr<pid_controller_msgs::srv::SetReference::Request> request,
         std::shared_ptr<pid_controller_msgs::srv::SetReference::Response> response) {

    request->request;
    // få tilgang til verdien fra requesten
    double requested_reference = request->request;

    // sjekk om verdien er gyldig
    if (requested_reference >= -M_PI && requested_reference <= M_PI) {
      // reference_ = requested_reference; // oppdaterer referansen
      pidController_.setReference(requested_reference); // oppdaterer referansen via PID-kontrolleren
      response->success = true;         // returnerer true hvis gyldig
      RCLCPP_INFO(this->get_logger(), "Valid reference received : %f. Updated reference :)", requested_reference);
    }
    else {
      response->success = false;        // returnerer false hvis ugyldig
      RCLCPP_WARN(this->get_logger(), "Invalid reference received : %f. NO UPDATE.", requested_reference);
    }

    // for debug
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Requested ref %f", requested_reference);
  }


public:
  void readParam();

  PIDControllerNode()
  : Node("pid_controller_node"), pidController_(1.0, 0.1, 0.05, 0.0) {


    // publisher for voltage
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("Lab1KineaVoltage", 10);

    // subscriber for measured_angle
    subscriber_ = this->create_subscription<std_msgs::msg::Float64>
                  ("Lab1KineaAngle", 10, std::bind(&PIDControllerNode::measurement_listener, this, std::placeholders::_1));

    // service node for set_reference
    service_ = this->create_service<pid_controller_msgs::srv::SetReference>
               ("set_reference", std::bind(&PIDControllerNode::handle_request_reference, this, std::placeholders::_1, std::placeholders::_2));


    RCLCPP_INFO(this->get_logger(), "Service 'set_reference' is ready.");
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



// main
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = std::make_shared<PIDControllerNode>();

  //rclcpp::Service<pid_controller_msgs::srv::SetReference>::SharedPtr set_reference_srv =
  //    node->create_service<pid_controller_msgs::srv::SetReference>("set_reference", &handle_request_reference);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


