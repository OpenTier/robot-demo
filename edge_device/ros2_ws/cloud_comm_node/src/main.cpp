#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "cloud_comm_node/cloud_comm_node.hpp"

int main(int argc, char * argv[])
{
  // Initialize ROS 2.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cloud_comm_node::CloudCommNode>();

  // Spin to process callbacks.
  rclcpp::spin(node);

  // Shutdown when done.
  rclcpp::shutdown();
  return 0;
}
