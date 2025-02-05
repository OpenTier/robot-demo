#include <stdio.h>
#include <unistd.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "Robot.h"
#include "SerialController.h"
#include "HttpController.h"
#include "RosController.h"
#include "WifiCom.h"

static const gpio_num_t SOLENOID_GATE_PIN = GPIO_NUM_21;
static const gpio_num_t PWM_LEFT_FRONT = GPIO_NUM_1;
static const gpio_num_t PWM_LEFT_REAR = GPIO_NUM_2;
static const gpio_num_t PWM_RIGHT_FRONT = GPIO_NUM_42;
static const gpio_num_t PWM_RIGHT_REAR = GPIO_NUM_41;
static const gpio_num_t WHITE_LED = GPIO_NUM_5;
static const gpio_num_t RED_LED = GPIO_NUM_6;

static const adc_unit_t BATTERY_ADC_UNIT = ADC_UNIT_1;
static const adc_channel_t BATTERY_ADC_CHANNEL = ADC_CHANNEL_3;
static const float BATTERY_DIVIDER = 5.0f;  // 5:1 voltage divider
static const int KICK_DURATION_MS = 50;

static const char* TAG = "Main";

void setup_controller(Robot& robot)
{
  // Control the robot using a Serial interface
  esp_err_t err;
#ifdef SERIAL_CONTROLLER
  SerialController controller(robot);
  err = controller.init();
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "Serial controller initialization failed.");
    return;
  }
  ESP_LOGI(TAG, "Serial initialized successfully.");

  while (1)
  {
    controller.update();
    vTaskDelay(pdMS_TO_TICKS(50));  // Adjust delay as needed.
  }
#elif HTTP_CONTROLLER
  // Control the robot using an HTTP controller
  HttpController httpController(robot);
  err = httpController.init();
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "ROS controller initialization failed.");
    return;
  }
  ESP_LOGI(TAG, "ROS controller initialized successfully.");

  while (1)
  {
    httpController.update();
    vTaskDelay(pdMS_TO_TICKS(50));  // Adjust delay as needed.
  }
#elif UROS_CONTROLLER
  RosController rosController(robot);
  err = rosController.init();
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "ROS controller initialization failed restarting the ESP32.");
    esp_restart();
    return;
  }
  ESP_LOGI(TAG, "ROS controller initialized successfully.");

  const int DELAY_TIME_MS = 25;
  // 1 second = 1000 ms / DELAY_TIME_MS
  const int COUNTER_LIMIT = 1000 / DELAY_TIME_MS;
  int counter = 0;

  while (1)
  {
    rosController.update();
    vTaskDelay(pdMS_TO_TICKS(DELAY_TIME_MS));
    counter++;
    if (counter >= COUNTER_LIMIT)
    {
      rosController.publish_status();
      counter = 0;
    }
  }
#else
#error "No controller defined."
#endif
}

extern "C" void app_main(void)
{
  ESP_LOGI(TAG, "Starting OpenTier T1 Robot ...");

  esp_err_t err;
#ifndef UROS_CONTROLLER
  // Initialize Wi - Fi
  ESP_LOGI(TAG, "Starting WiFi Initialization ...");
  err = WifiComInit();
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "WiFi initialization failed.");
    return;
  }
  ESP_LOGI(TAG, "WiFi initialized successfully.");
#endif

  // Create Robot instance using constructor parameters
  Robot robot(PWM_LEFT_FRONT, PWM_LEFT_REAR, PWM_RIGHT_FRONT, PWM_RIGHT_REAR, SOLENOID_GATE_PIN, KICK_DURATION_MS,
              BATTERY_ADC_UNIT, BATTERY_ADC_CHANNEL, BATTERY_DIVIDER, RED_LED, WHITE_LED, 50);
  // without Robot HW this ends up in infinite loop so you can build the firmware with this flag
  // to skip init phase and use only communication
#ifndef SKIP_ROBOT_INIT
  err = robot.init();
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "Robot initialization failed.");
    return;
  }
  ESP_LOGI(TAG, "Robot initialized successfully.");
#endif

  setup_controller(robot);
}
