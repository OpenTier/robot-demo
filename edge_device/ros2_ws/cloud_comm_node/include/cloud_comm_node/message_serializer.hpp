#pragma once

#include <string>
#include "robot_soccer.pb.h"
#include "edge_device_msgs/msg/beep.hpp"
#include "edge_device_msgs/msg/kick.hpp"
#include "edge_device_msgs/msg/leds.hpp"
#include "edge_device_msgs/msg/move.hpp"
#include "edge_device_msgs/msg/robot_status.hpp"
#include "vehicle_cloud_events.pb.h"
#include <optional>

namespace cloud_comm_node
{

namespace edge_msgs = ::edge_device_msgs::msg;

class MessageSerializer
{
public:
  // Convert a ROS telemetry message to a protobuf RobotPacket (status)
  static std::optional<std::string> serializeStatus(const edge_msgs::RobotStatus& msg)
  {
    robot::RobotPacket packet;
    packet.set_robot_id(msg.robot_id);
    auto* status = packet.mutable_status();
    status->set_timestamp(msg.timestamp);
    status->set_is_running(msg.is_running);
    auto* battery_event = status->mutable_battery_info();
    battery_event->set_vehicle_id(std::to_string(msg.robot_id));
    battery_event->set_is_discharging(true);
    battery_event->set_is_charging(false);
    battery_event->set_battery_level(msg.voltage);

    std::string serialized_data;
    if (!packet.SerializeToString(&serialized_data))
    {
      return std::nullopt;
    }

    return serialized_data;
  }

  // Deserialize raw data from Zenoh into a RobotPacket
  static std::optional<robot::RobotPacket> deserializePacket(const std::string& data)
  {
    robot::RobotPacket packet;
    if (!packet.ParseFromString(data))
    {
      return std::nullopt;
    }
    return packet;
  }

  static edge_msgs::Beep convertToBeepCommand(const robot::RobotCommand& proto_cmd, const uint32_t robot_id)
  {
    (void)proto_cmd;
    edge_msgs::Beep beep_msg;
    beep_msg.robot_id = robot_id;
    beep_msg.frequency = proto_cmd.beep().frequency();
    beep_msg.duration = proto_cmd.beep().duration();
    return beep_msg;
  }

  static edge_msgs::Kick convertToKickCommand(const robot::RobotCommand& proto_cmd, const uint32_t robot_id)
  {
    edge_msgs::Kick kick_msg;
    kick_msg.robot_id = robot_id;
    kick_msg.power = proto_cmd.kick().power();

    return kick_msg;
  }

  static edge_msgs::Leds convertToLedsCommand(const robot::RobotCommand& proto_cmd, const uint32_t robot_id)
  {
    edge_msgs::Leds leds_msg;
    leds_msg.robot_id = robot_id;
    leds_msg.blue = proto_cmd.leds().blue();
    leds_msg.green = proto_cmd.leds().green();
    leds_msg.red = proto_cmd.leds().red();

    return leds_msg;
  }

  static edge_msgs::Move convertToMoveCommand(const robot::RobotCommand& proto_cmd, const uint32_t robot_id)
  {
    edge_msgs::Move move_msg;
    move_msg.robot_id = robot_id;
    move_msg.dturn = proto_cmd.control().dturn();
    move_msg.dx = proto_cmd.control().dx();
    move_msg.dy = proto_cmd.control().dy();

    return move_msg;
  }
};

}  // namespace cloud_comm_node