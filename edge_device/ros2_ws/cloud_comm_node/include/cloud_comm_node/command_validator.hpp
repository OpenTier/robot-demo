#pragma once

#include "robot_soccer.pb.h"
#include "cloud_comm_node/configuration.hpp"

namespace cloud_comm_node
{

  class CommandValidator
  {
  public:
    static bool validate(const robot::RobotPacket &packet)
    {
      // TODO: add some validations rule here
      (void)packet;
      return true;
    }
  };

} // namespace cloud_comm_node