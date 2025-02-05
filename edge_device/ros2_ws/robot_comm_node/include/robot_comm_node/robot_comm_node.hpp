#pragma once

#include "rclcpp/rclcpp.hpp"
#include "edge_device_msgs/msg/beep.hpp"
#include "edge_device_msgs/msg/kick.hpp"
#include "edge_device_msgs/msg/leds.hpp"
#include "edge_device_msgs/msg/move.hpp"
#include "edge_device_msgs/msg/robot_status.hpp"
#include "robot_msgs/msg/robot_cmd.hpp"
#include "robot_msgs/msg/robot_status.hpp"

#include <map>
#include <cstdint>

class RobotCommNode : public rclcpp::Node
{
public:
  RobotCommNode();

private:
  // Callback for the "beep" command.
  void beep_callback(const edge_device_msgs::msg::Beep::SharedPtr msg);

  // Callback for the "kick" command.
  void kick_callback(const edge_device_msgs::msg::Kick::SharedPtr msg);

  // Callback for the "leds" command.
  void leds_callback(const edge_device_msgs::msg::Leds::SharedPtr msg);

  // Callback for the "move" command.
  void move_callback(const edge_device_msgs::msg::Move::SharedPtr msg);

  // Callback to handle robot status messages and forward them to cloud_comm_node.
  void robot_status_callback(uint32_t robot_id, const robot_msgs::msg::RobotStatus::SharedPtr msg);

  // Subscribers for incoming commands.
  rclcpp::Subscription<edge_device_msgs::msg::Beep>::SharedPtr m_beep_sub;
  rclcpp::Subscription<edge_device_msgs::msg::Kick>::SharedPtr m_kick_sub;
  rclcpp::Subscription<edge_device_msgs::msg::Leds>::SharedPtr m_leds_sub;
  rclcpp::Subscription<edge_device_msgs::msg::Move>::SharedPtr m_move_sub;

  // Publisher for robot status forwarded to cloud_comm_node.
  rclcpp::Publisher<edge_device_msgs::msg::RobotStatus>::SharedPtr m_status_pub;

  // Map of publishers for sending commands to each robot.
  std::map<uint32_t, rclcpp::Publisher<robot_msgs::msg::RobotCmd>::SharedPtr> m_robot_cmd_publishers;

  // Map of subscribers for each robot's status updates.
  std::map<uint32_t, rclcpp::Subscription<robot_msgs::msg::RobotStatus>::SharedPtr> m_robot_status_subscribers;
};
