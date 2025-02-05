#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <cloud_comm_node/zenoh_client.hpp>
#include <cloud_comm_node/configuration.hpp>
#include <cloud_comm_node/telemetry_bridge.hpp>
#include <cloud_comm_node/command_bridge.hpp>

namespace cloud_comm_node
{

  class CloudCommNode : public rclcpp::Node
  {
  public:
    CloudCommNode() : Node("cloud_comm_node")
    {
      m_zenoh_client = std::make_shared<ZenohClient>();
      m_telemetry_bridge = std::make_shared<TelemetryBridge>(this, m_zenoh_client);
      m_command_bridge = std::make_shared<CommandBridge>(this, m_zenoh_client);
      RCLCPP_INFO(this->get_logger(), "Cloud Communication Node is running");
    }

  private:
    std::shared_ptr<ZenohClient> m_zenoh_client;
    std::shared_ptr<TelemetryBridge> m_telemetry_bridge;
    std::shared_ptr<CommandBridge> m_command_bridge;
  };

} // namespace cloud_comm_node
