#include "robot_comm_node/robot_comm_node.hpp"
#include <cstring>  // For memcpy and memset

RobotCommNode::RobotCommNode() : Node("robot_comm_node")
{
  // Create subscribers for commands coming from cloud_comm_node.
  m_beep_sub = this->create_subscription<edge_device_msgs::msg::Beep>(
      "robot/beep", 10, std::bind(&RobotCommNode::beep_callback, this, std::placeholders::_1));

  m_kick_sub = this->create_subscription<edge_device_msgs::msg::Kick>(
      "robot/kick", 10, std::bind(&RobotCommNode::kick_callback, this, std::placeholders::_1));

  m_leds_sub = this->create_subscription<edge_device_msgs::msg::Leds>(
      "robot/leds", 10, std::bind(&RobotCommNode::leds_callback, this, std::placeholders::_1));

  m_move_sub = this->create_subscription<edge_device_msgs::msg::Move>(
      "robot/move", 10, std::bind(&RobotCommNode::move_callback, this, std::placeholders::_1));

  // Publisher to forward robot status to cloud_comm_node.
  m_status_pub = this->create_publisher<edge_device_msgs::msg::RobotStatus>("robot_status", 10);

  // Assuming two robots with IDs 1 and 2, create publishers for their command topics.
  m_robot_cmd_publishers[1] = this->create_publisher<robot_msgs::msg::RobotCmd>("robot1/cmd", 10);
  m_robot_cmd_publishers[2] = this->create_publisher<robot_msgs::msg::RobotCmd>("robot2/cmd", 10);

  // Create subscribers for each robot's status topic.
  m_robot_status_subscribers[1] = this->create_subscription<robot_msgs::msg::RobotStatus>(
      "robot1/status", 10,
      [this](const robot_msgs::msg::RobotStatus::SharedPtr msg) { this->robot_status_callback(1, msg); });

  m_robot_status_subscribers[2] = this->create_subscription<robot_msgs::msg::RobotStatus>(
      "robot2/status", 10,
      [this](const robot_msgs::msg::RobotStatus::SharedPtr msg) { this->robot_status_callback(2, msg); });

  RCLCPP_INFO(this->get_logger(), "RobotCommNode started");
}

// Callback for the "beep" command.
void RobotCommNode::beep_callback(const edge_device_msgs::msg::Beep::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received beep command for robot %d", msg->robot_id);

  auto robot_id = msg->robot_id;
  robot_msgs::msg::RobotCmd cmd;

  cmd.command_id = robot_msgs::msg::RobotCmd::COMMAND_BEEP;
  uint32_t frequency = msg->frequency;
  uint32_t duration = msg->duration;

  // Pack the frequency and duration into the first 8 bytes of the 12-byte payload.
  memcpy(cmd.payload.data(), &frequency, sizeof(uint32_t));
  memcpy(cmd.payload.data() + sizeof(uint32_t), &duration, sizeof(uint32_t));

  if (m_robot_cmd_publishers.find(robot_id) != m_robot_cmd_publishers.end())
  {
    m_robot_cmd_publishers[robot_id]->publish(cmd);
    RCLCPP_INFO(this->get_logger(), "Published beep command for robot %d", robot_id);
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Received beep command for unknown robot_id: %d", robot_id);
  }
}

// Callback for the "kick" command.
void RobotCommNode::kick_callback(const edge_device_msgs::msg::Kick::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received kick command for robot %d", msg->robot_id);

  auto robot_id = msg->robot_id;
  robot_msgs::msg::RobotCmd cmd;

  cmd.command_id = robot_msgs::msg::RobotCmd::COMMAND_KICK;
  float power = msg->power;
  // Pack the power value into the first 4 bytes of the payload.
  memcpy(cmd.payload.data(), &power, sizeof(float));

  if (m_robot_cmd_publishers.find(robot_id) != m_robot_cmd_publishers.end())
  {
    m_robot_cmd_publishers[robot_id]->publish(cmd);
    RCLCPP_INFO(this->get_logger(), "Published kick command for robot %d", robot_id);
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Received kick command for unknown robot_id: %d", robot_id);
  }
}

// Callback for the "leds" command.
void RobotCommNode::leds_callback(const edge_device_msgs::msg::Leds::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received leds command for robot %d", msg->robot_id);

  auto robot_id = msg->robot_id;
  robot_msgs::msg::RobotCmd cmd;

  cmd.command_id = robot_msgs::msg::RobotCmd::COMMAND_LEDS;
  // Pack red, green, and blue into the first 3 bytes of the payload.
  cmd.payload[0] = msg->red;
  cmd.payload[1] = msg->green;
  cmd.payload[2] = msg->blue;

  if (m_robot_cmd_publishers.find(robot_id) != m_robot_cmd_publishers.end())
  {
    m_robot_cmd_publishers[robot_id]->publish(cmd);
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Received leds command for unknown robot_id: %d", robot_id);
  }
}

// Callback for the "move" command.
void RobotCommNode::move_callback(const edge_device_msgs::msg::Move::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received move command for robot %d", msg->robot_id);

  auto robot_id = msg->robot_id;
  robot_msgs::msg::RobotCmd cmd;

  cmd.command_id = robot_msgs::msg::RobotCmd::COMMAND_MOVE;
  float dx = msg->dx;
  float dy = msg->dy;
  float dturn = msg->dturn;

  // Pack the three float values into the 12-byte payload.
  memcpy(cmd.payload.data(), &dx, sizeof(float));
  memcpy(cmd.payload.data() + sizeof(float), &dy, sizeof(float));
  memcpy(cmd.payload.data() + 2 * sizeof(float), &dturn, sizeof(float));

  if (m_robot_cmd_publishers.find(robot_id) != m_robot_cmd_publishers.end())
  {
    m_robot_cmd_publishers[robot_id]->publish(cmd);
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Received move command for unknown robot_id: %d", robot_id);
  }
}

// Callback to handle robot status messages and forward them to cloud_comm_node.
void RobotCommNode::robot_status_callback(uint32_t robot_id, const robot_msgs::msg::RobotStatus::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received status update for robot %d", robot_id);
  edge_device_msgs::msg::RobotStatus status;
  status.robot_id = robot_id;
  status.voltage = msg->voltage;
  status.is_running = true;
  // TODO: we don't have timestamp in float; here we use robot_id as a placeholder.
  status.timestamp = robot_id;

  m_status_pub->publish(status);
}
