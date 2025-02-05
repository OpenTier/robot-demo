#include "rclcpp/rclcpp.hpp"

#include "robot_comm_node/robot_comm_node.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotCommNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
