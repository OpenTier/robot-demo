#pragma once

#include "IController.h"
#include "Robot.h"

// Include FreeRTOS headers for tick count functions
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// micro-ROS includes
extern "C" {
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <robot_msgs/msg/robot_status.h>
#include <robot_msgs/msg/robot_cmd.h>
}

class RosController : public IController
{
public:
  explicit RosController(Robot& r);
  virtual esp_err_t init() override;
  virtual void update() override;
  ~RosController();

  void publish_status();

private:
  // Callback for received messages on the command topic
  static void subscription_callback(const void* msgin);

  // Process the received command string
  void process_command(uint8_t command_id, const uint8_t* data);

  Robot& robot;

  // micro-ROS entities
  rcl_node_t node;
  rcl_publisher_t publisher;
  rcl_subscription_t subscriber;
  rclc_executor_t executor;
  robot_msgs__msg__RobotStatus tx_msg;
  robot_msgs__msg__RobotCmd rx_msg;
  rcl_allocator_t allocator;
  rcl_init_options_t init_options;

  // Store the support structure so its context remains valid
  rclc_support_t support;

  // Keep track of whether initialization succeeded
  bool initialized;

  uint8_t error_counter;

  // Store the tick count of the last received command
  TickType_t last_command_tick;

  // Static instance pointer for callback context (assumes a single instance)
  static RosController* instance;
};
