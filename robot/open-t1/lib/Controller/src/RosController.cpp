#include "RosController.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include <uros_network_interfaces.h>
#include <rmw_microros/rmw_microros.h>

#ifndef CONFIG_MICRO_ROS_AGENT_IP
#define CONFIG_MICRO_ROS_AGENT_IP "192.168.50.39"
#endif

#ifndef CONFIG_MICRO_ROS_AGENT_PORT
#define CONFIG_MICRO_ROS_AGENT_PORT 8888
#endif

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#ifndef DOMAIN_ID
#define DOMAIN_ID 42
#endif

#ifndef ROBOT_ID
#define ROBOT_ID 1
#endif

#ifndef STATUS_TOPIC
#define STATUS_TOPIC "robot" TOSTRING(ROBOT_ID) "/status"
#endif

#ifndef CMD_TOPIC
#define CMD_TOPIC "robot" TOSTRING(ROBOT_ID) "/cmd"
#endif

// Timeout for receiving new commands (in milliseconds)
// If robot doesn't receive a new command within this time, it will stop.
#ifndef COMMAND_TIMEOUT_MS
#define COMMAND_TIMEOUT_MS 1000
#endif

// Helper macros for error checking
#define RCCHECK(fn)                                                                                                    \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    if ((temp_rc != RCL_RET_OK))                                                                                       \
    {                                                                                                                  \
      ESP_LOGE(TAG, "Failed status on line %d: %d", __LINE__, (int)temp_rc);                                           \
      return ESP_FAIL;                                                                                                 \
    }                                                                                                                  \
  }

static const char* TAG = "RosController";

// Initialize the static instance pointer to nullptr.
RosController* RosController::instance = nullptr;

RosController::RosController(Robot& r) : robot(r), initialized(false), error_counter(0)
{
  // Set the static instance pointer to this object
  instance = this;

  // Initialize micro-ROS entities to zero
  node = rcl_get_zero_initialized_node();
  publisher = rcl_get_zero_initialized_publisher();
  subscriber = rcl_get_zero_initialized_subscription();
  executor = rclc_executor_get_zero_initialized_executor();
  memset(&rx_msg, 0, sizeof(rx_msg));
  memset(&tx_msg, 0, sizeof(tx_msg));
  allocator = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();

  // init with the current tick count
  last_command_tick = xTaskGetTickCount();
}

RosController::~RosController()
{
  // Clean up micro-ROS entities (error handling omitted for brevity)
  rcl_publisher_fini(&publisher, &node);
  rcl_subscription_fini(&subscriber, &node);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

esp_err_t RosController::init()
{
  esp_err_t err = uros_network_interface_initialize();
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to initialize network interface");
    return err;
  }
  // Initialize rcl init options and set up the domain ID.
  RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
  // Set the ROS Domain ID.
  RCCHECK(rcl_init_options_set_domain_id(&init_options, DOMAIN_ID));

  rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

  // Static Agent IP and port can be used instead of autodisvery.
  RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif

  // Initialize the support structure and store it as a member variable.
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // Create a micro-ROS node.
  RCCHECK(rclc_node_init_default(&node, "uros_robot", "", &support));

  // Create a publisher (e.g., for status messages).
  RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(robot_msgs, msg, RobotStatus),
                                      STATUS_TOPIC));

  // Create a subscriber for command messages.
  RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(robot_msgs, msg, RobotCmd),
                                         CMD_TOPIC));

  // Initialize the executor with one handle (for the subscriber).
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &rx_msg, &RosController::subscription_callback,
                                         ON_NEW_DATA));

  initialized = true;
  ESP_LOGI(TAG, "micro-ROS node initialized");

  return ESP_OK;
}

void RosController::update()
{
  if (!initialized)
  {
    ESP_LOGE(TAG, "RosController not initialized");
    return;
  }
  // Spin the executor to process incoming messages (non-blocking).
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  TickType_t current_tick = xTaskGetTickCount();
  if ((current_tick - last_command_tick) > pdMS_TO_TICKS(COMMAND_TIMEOUT_MS))
  {
    ESP_LOGI(TAG, "Command not received for %d ms, stopping robot", COMMAND_TIMEOUT_MS);
    robot.stop();
    last_command_tick = current_tick;
  }
}

void RosController::publish_status()
{
  if (!initialized)
  {
    ESP_LOGE(TAG, "RosController not initialized");
    return;
  }
  ESP_LOGI(TAG, "Publishing status message");
  // Prepare a status message and publish it.
  tx_msg.voltage = robot.getBatteryVoltage();

  rcl_ret_t err = rcl_publish(&publisher, &tx_msg, NULL);
  if ((err != RCL_RET_OK))
  {
    ESP_LOGW(TAG, "Couldn't pubilsh status message");
    error_counter++;
    if (error_counter >= 5)
    {
      ESP_LOGW(TAG, "Too many errors, restarting board");
      esp_restart();
    }
  }
}

void RosController::subscription_callback(const void* msgin)
{
  ESP_LOGI(TAG, "Received command.");
  const robot_msgs__msg__RobotCmd* received = (const robot_msgs__msg__RobotCmd*)msgin;

  // Use the static instance pointer for callback context.
  if (instance != nullptr)
  {
    instance->last_command_tick = xTaskGetTickCount();
    instance->process_command(received->command_id, received->payload);
  }
}

void RosController::process_command(uint8_t command_id, const uint8_t* payload)
{
  ESP_LOGI(TAG, "Processing command: %d", command_id);

  switch (command_id)
  {
    case robot_msgs__msg__RobotCmd__COMMAND_MOVE: {
      // The MOVE command payload is 12 bytes:
      //   - dx (float, m/s)
      //   - dy (float, m/s)
      //   - dturn (float, rad/s)
      float dx, dy, dturn;
      memcpy(&dx, payload, sizeof(float));
      memcpy(&dy, payload + sizeof(float), sizeof(float));
      memcpy(&dturn, payload + 2 * sizeof(float), sizeof(float));

      // Execute the move command.
      ESP_LOGI(TAG, "Move: dx=%f, dy=%f, dturn=%f", dx, dy, dturn);
      robot.controlVelocity(dx, dy, dturn);
    }
    break;
    case robot_msgs__msg__RobotCmd__COMMAND_BEEP: {
      // The BEEP command payload is 8 bytes:
      //   - frequency (uint32_t, Hz)
      //   - duration (uint32_t, milliseconds)
      // The remaining 4 bytes are unused.
      uint32_t frequency, duration;
      memcpy(&frequency, payload, sizeof(uint32_t));
      memcpy(&duration, payload + sizeof(uint32_t), sizeof(uint32_t));

      ESP_LOGI(TAG, "Beep: frequency=%d, duration=%d", frequency, duration);
    }
    break;
    case robot_msgs__msg__RobotCmd__COMMAND_KICK: {
      // The KICK command payload is 4 bytes:
      //   - power (float, value from 0.0 to 1.0)
      // The remaining 8 bytes are unused.
      float power;
      memcpy(&power, payload, sizeof(float));
      power = power * 1000;
      ESP_LOGI(TAG, "Kick: power=%f", power);
      robot.kick(power);
    }
    break;
    case robot_msgs__msg__RobotCmd__COMMAND_LEDS: {
      // The LEDS command payload is 3 bytes:
      //   - red (uint8_t, 0-255)
      //   - green (uint8_t, 0-255)
      //   - blue (uint8_t, 0-255)
      // The remaining 9 bytes are unused.
      uint8_t red = payload[0];
      uint8_t green = payload[1];
      uint8_t blue = payload[2];
      ESP_LOGI(TAG, "LEDs: R=%d, G=%d, B=%d", red, green, blue);
    }
    break;
    default:
      ESP_LOGW(TAG, "Unknown command: %d", command_id);
      break;
  }
}
