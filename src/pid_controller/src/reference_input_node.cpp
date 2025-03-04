#include "rclcpp/rclcpp.hpp"
#include "pid_controller_msgs/srv/set_reference.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  /*if (argc != 2) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: set PID reference X");
      return 1;
  }*/
  
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("set_reference_client");
  rclcpp::Client<pid_controller_msgs::srv::SetReference>::SharedPtr client =
    node->create_client<pid_controller_msgs::srv::SetReference>("set_reference");

  auto request = std::make_shared<pid_controller_msgs::srv::SetReference::Request>();
  //request->request = atoll(argv[1]);
  //std::string line;
  //std::getline( std::cin, line );
  request->request = rand()%5;
  
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result: %d", result.get()->success);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service set_reference");
  }

  rclcpp::shutdown();
  return 0;
}
