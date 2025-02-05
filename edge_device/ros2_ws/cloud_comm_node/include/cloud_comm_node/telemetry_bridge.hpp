#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <cloud_comm_node/zenoh_client.hpp>
#include <cloud_comm_node/configuration.hpp>
#include <cloud_comm_node/message_serializer.hpp>
#include "robot_soccer.pb.h"
#include "edge_device_msgs/msg/robot_status.hpp"

namespace cloud_comm_node
{

class TelemetryBridge
{
public:
  TelemetryBridge(rclcpp::Node* node, std::shared_ptr<ZenohClient> zenoh_client)
    : m_node(node), m_zenoh_client(zenoh_client)
  {
    m_telemetry_sub = m_node->create_subscription<edge_device_msgs::msg::RobotStatus>(
        "robot_status", 10, std::bind(&TelemetryBridge::telemetryCallback, this, std::placeholders::_1));
    RCLCPP_INFO(m_node->get_logger(), "Telemetry Bridge is running");
  }

private:
  void telemetryCallback(const edge_device_msgs::msg::RobotStatus& msg)
  {
    RCLCPP_INFO(m_node->get_logger(), "Received telemetry message");

    // Convert ROS message to protobuf using MessageSerializer.
    auto packet = MessageSerializer::serializeStatus(msg);

    if (!packet)
    {
      RCLCPP_ERROR(m_node->get_logger(), "Failed to serialize RobotStatus message");
      return;
    }
    // Publish to cloud via Zenoh.
    m_zenoh_client->publish_robot_status(*packet);
    RCLCPP_INFO(m_node->get_logger(), "Robot status published to cloud");
  }

  rclcpp::Node* m_node;
  std::shared_ptr<ZenohClient> m_zenoh_client;
  rclcpp::Subscription<edge_device_msgs::msg::RobotStatus>::SharedPtr m_telemetry_sub;
};

}  // namespace cloud_comm_node