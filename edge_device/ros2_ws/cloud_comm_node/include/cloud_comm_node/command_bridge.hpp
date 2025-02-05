#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <cloud_comm_node/zenoh_client.hpp>
#include <cloud_comm_node/configuration.hpp>
#include <cloud_comm_node/command_validator.hpp>
#include <cloud_comm_node/message_serializer.hpp>
#include "robot_soccer.pb.h"

#include "edge_device_msgs/msg/beep.hpp"
#include "edge_device_msgs/msg/kick.hpp"
#include "edge_device_msgs/msg/leds.hpp"
#include "edge_device_msgs/msg/move.hpp"

namespace cloud_comm_node
{

namespace edge_msgs = ::edge_device_msgs::msg;

class CommandBridge
{
public:
  CommandBridge(rclcpp::Node* node, std::shared_ptr<ZenohClient> zenoh_client)
    : m_node(node), m_zenoh_client(zenoh_client)
  {
    m_beep_pub = m_node->create_publisher<edge_msgs::Beep>("robot/beep", 10);
    m_kick_pub = m_node->create_publisher<edge_msgs::Kick>("robot/kick", 10);
    m_leds_pub = m_node->create_publisher<edge_msgs::Leds>("robot/leds", 10);
    m_move_pub = m_node->create_publisher<edge_msgs::Move>("robot/move", 10);

    // subscribe to robot 1 commands
    m_zenoh_client->subscribe_to_robot_command("1", [this](const std::string& data) {
      RCLCPP_INFO(m_node->get_logger(), "Received command: %s", data.c_str());
      commandCallback(data);
    });
    // subscribe to robot 2 commands
    m_zenoh_client->subscribe_to_robot_command("2", [this](const std::string& data) {
      RCLCPP_INFO(m_node->get_logger(), "Received command: %s", data.c_str());
      commandCallback(data);
    });

    RCLCPP_INFO(m_node->get_logger(), "Command Bridge is running");
  }

private:
  void commandCallback(const std::string& raw_packet)
  {
    // Deserialize the raw data into a protobuf packet.
    auto maybe_packet = MessageSerializer::deserializePacket(raw_packet);

    if (!maybe_packet)
    {
      RCLCPP_ERROR(m_node->get_logger(), "Failed to deserialize RobotPacket message");
      return;
    }

    auto& packet = *maybe_packet;

    // validate packet
    if (!CommandValidator::validate(packet))
    {
      RCLCPP_WARN(m_node->get_logger(), "Invalid command received");
      return;
    }

    const uint32_t robot_id = packet.robot_id();

    using CommandType = robot::RobotCommand::CommandCase;

    // dispatch command
    switch (packet.command().command_case())
    {
      case CommandType::kControl: {
        edge_msgs::Move move = MessageSerializer::convertToMoveCommand(packet.command(), robot_id);
        m_move_pub->publish(move);
      }
      break;
      case CommandType::kKick: {
        edge_msgs::Kick kick = MessageSerializer::convertToKickCommand(packet.command(), robot_id);
        m_kick_pub->publish(kick);
      }
      break;
      case CommandType::kLeds: {
        edge_msgs::Leds leds = MessageSerializer::convertToLedsCommand(packet.command(), robot_id);
        m_leds_pub->publish(leds);
      }
      break;
      case CommandType::kBeep: {
        edge_msgs::Beep beep = MessageSerializer::convertToBeepCommand(packet.command(), robot_id);
        m_beep_pub->publish(beep);
      }
      break;
      case CommandType::COMMAND_NOT_SET:  // fallthrough
      default:
        RCLCPP_WARN(m_node->get_logger(), "Unknown command received");
        break;
    }
  }

  rclcpp::Node* m_node;
  std::shared_ptr<ZenohClient> m_zenoh_client;
  rclcpp::Publisher<edge_msgs::Beep>::SharedPtr m_beep_pub;
  rclcpp::Publisher<edge_msgs::Kick>::SharedPtr m_kick_pub;
  rclcpp::Publisher<edge_msgs::Leds>::SharedPtr m_leds_pub;
  rclcpp::Publisher<edge_msgs::Move>::SharedPtr m_move_pub;
};

}  // namespace cloud_comm_node