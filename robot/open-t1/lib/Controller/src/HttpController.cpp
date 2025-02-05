#include "HttpController.h"
#include <esp_log.h>
#include <esp_http_server.h>
#include <cstring>
#include <cstdlib>
#include <cctype>

const char* HttpController::TAG = "HttpController";

HttpController::HttpController(Robot& r) : robot(r), serverHandle(nullptr)
{
}

esp_err_t HttpController::init()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.uri_match_fn = httpd_uri_match_wildcard;
  config.lru_purge_enable = true;

  ESP_LOGI(TAG, "Starting HTTP server on port %d", config.server_port);
  esp_err_t err = httpd_start(&serverHandle, &config);
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(err));
    return err;
  }

  // Register /cmd
  httpd_uri_t cmdUri = { .uri = "/cmd", .method = HTTP_GET, .handler = &HttpController::cmdHandler, .user_ctx = this };
  httpd_register_uri_handler(serverHandle, &cmdUri);

  return ESP_OK;
}

void HttpController::update()
{
  // Plain HTTP is event-driven, so nothing special here.
}

esp_err_t HttpController::cmdHandler(httpd_req_t* req)
{
  auto* that = static_cast<HttpController*>(req->user_ctx);

  char query[128] = { 0 };
  esp_err_t err = httpd_req_get_url_query_str(req, query, sizeof(query));
  if (err != ESP_OK)
  {
    ESP_LOGW(TAG, "No query or error retrieving it: %s", esp_err_to_name(err));
    const char* msg = "{\"status\":\"error\",\"message\":\"No query parameters\"}";
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, msg);
    ESP_LOGI(TAG, "Response: %s", msg);
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Got query: %s", query);

  // Parse out all possible parameters
  char cmdBuf[32] = { 0 };
  char speedBuf[16] = { 0 };
  char durBuf[16] = { 0 };
  char xBuf[16] = { 0 };
  char yBuf[16] = { 0 };
  char rotBuf[16] = { 0 };
  char intervalBuf[16] = { 0 };
  char dutyBuf[16] = { 0 };
  char timesBuf[16] = { 0 };

  httpd_query_key_value(query, "command", cmdBuf, sizeof(cmdBuf));
  httpd_query_key_value(query, "speed", speedBuf, sizeof(speedBuf));
  httpd_query_key_value(query, "duration", durBuf, sizeof(durBuf));
  httpd_query_key_value(query, "x", xBuf, sizeof(xBuf));
  httpd_query_key_value(query, "y", yBuf, sizeof(yBuf));
  httpd_query_key_value(query, "rot", rotBuf, sizeof(rotBuf));
  httpd_query_key_value(query, "interval", intervalBuf, sizeof(intervalBuf));
  httpd_query_key_value(query, "duty", dutyBuf, sizeof(dutyBuf));
  httpd_query_key_value(query, "times", timesBuf, sizeof(timesBuf));

  int speedVal = -1;
  if (speedBuf[0] != '\0')
  {
    speedVal = atoi(speedBuf);
  }
  int durVal = -1;
  if (durBuf[0] != '\0')
  {
    durVal = atoi(durBuf);
  }

  float xVal_f = (xBuf[0] != '\0') ? strtof(xBuf, nullptr) : 0.0f;
  float yVal_f = (yBuf[0] != '\0') ? strtof(yBuf, nullptr) : 0.0f;
  float rotVal_f = (rotBuf[0] != '\0') ? strtof(rotBuf, nullptr) : 0.0f;

  int intervalVal = 0;
  if (intervalBuf[0] != '\0')
  {
    intervalVal = atoi(intervalBuf);
  }
  float dutyVal = 50.0f;
  if (dutyBuf[0] != '\0')
  {
    dutyVal = strtof(dutyBuf, nullptr);
  }
  int timesVal = 0;  // default infinite
  if (timesBuf[0] != '\0')
  {
    timesVal = atoi(timesBuf);
  }

  std::string command(cmdBuf);

  // Handle the command
  std::string response =
      that->handleCommand(command, speedVal, durVal, xVal_f, yVal_f, rotVal_f, intervalVal, dutyVal, timesVal);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr(req, response.c_str());

  ESP_LOGI(TAG, "Response: %s", response.c_str());
  return ESP_OK;
}

std::string HttpController::handleCommand(const std::string& command, int speed, int durationMs, float xVal, float yVal,
                                          float rotVal, int interval, float duty, int times)
{
  // Lowercase the command
  std::string cmd = command;
  for (auto& c : cmd)
  {
    c = static_cast<char>(tolower((unsigned char)c));
  }

  std::string response;

  // ------------------
  // Movement / Kicker / Battery
  // ------------------
  if (cmd == "forward")
  {
    if (!checkSpeedRange(speed))
      response = "{\"status\":\"error\",\"message\":\"Invalid speed (0..100 or -1)\"}";
    else
    {
      robot.moveForward(speed);
      response = "{\"status\":\"OK\",\"action\":\"moveForward\"}";
    }
  }
  else if (cmd == "backward")
  {
    if (!checkSpeedRange(speed))
      response = "{\"status\":\"error\",\"message\":\"Invalid speed\"}";
    else
    {
      robot.moveBackward(speed);
      response = "{\"status\":\"OK\",\"action\":\"moveBackward\"}";
    }
  }
  else if (cmd == "left")
  {
    if (!checkSpeedRange(speed))
      response = "{\"status\":\"error\",\"message\":\"Invalid speed\"}";
    else
    {
      robot.moveLeft(speed);
      response = "{\"status\":\"OK\",\"action\":\"moveLeft\"}";
    }
  }
  else if (cmd == "right")
  {
    if (!checkSpeedRange(speed))
      response = "{\"status\":\"error\",\"message\":\"Invalid speed\"}";
    else
    {
      robot.moveRight(speed);
      response = "{\"status\":\"OK\",\"action\":\"moveRight\"}";
    }
  }
  else if (cmd == "rotateleft")
  {
    if (!checkSpeedRange(speed))
      response = "{\"status\":\"error\",\"message\":\"Invalid speed\"}";
    else
    {
      robot.rotateLeft(speed);
      response = "{\"status\":\"OK\",\"action\":\"rotateLeft\"}";
    }
  }
  else if (cmd == "rotateright")
  {
    if (!checkSpeedRange(speed))
      response = "{\"status\":\"error\",\"message\":\"Invalid speed\"}";
    else
    {
      robot.rotateRight(speed);
      response = "{\"status\":\"OK\",\"action\":\"rotateRight\"}";
    }
  }
  else if (cmd == "stop")
  {
    robot.stop();
    response = "{\"status\":\"OK\",\"action\":\"stop\"}";
  }
  else if (cmd == "kick")
  {
    if (!checkDurationRange(durationMs))
      response = "{\"status\":\"error\",\"message\":\"Kick duration must be 0..2000 or -1\"}";
    else
    {
      if (durationMs < 0)
        robot.kick();
      else
        robot.kick(durationMs);
      response = "{\"status\":\"OK\",\"action\":\"kick\"}";
    }
  }
  else if (cmd == "battery")
  {
    float voltage = robot.getBatteryVoltage();
    char buff[64];
    snprintf(buff, sizeof(buff), "{\"status\":\"OK\",\"batteryVoltage\":%.2f}", voltage);
    response = buff;
  }
  else if (cmd == "normalizedcontrol")
  {
    // x,y,rot in [-100..100]
    if (xVal < -100 || xVal > 100 || yVal < -100 || yVal > 100 || rotVal < -100 || rotVal > 100)
    {
      response = "{\"status\":\"error\",\"message\":\"x,y,rot must be -100..100\"}";
    }
    else
    {
      robot.controlVelocityNormalized(xVal, yVal, rotVal);
      response = "{\"status\":\"OK\",\"action\":\"normalizedcontrol\"}";
    }
  }
  else if (cmd == "control")
  {
    // xVal,yVal in mm/s, rotVal in rad/s
    robot.controlVelocity(xVal, yVal, rotVal / 1000.0f);
    response = "{\"status\":\"OK\",\"action\":\"control\"}";
  }

  // ------------------
  // LED Commands
  // ------------------
  else if (cmd == "whiteon")
  {
    robot.turnOnWhiteLed();
    response = "{\"status\":\"OK\",\"action\":\"whiteon\"}";
  }
  else if (cmd == "whiteoff")
  {
    robot.turnOffWhiteLed();
    response = "{\"status\":\"OK\",\"action\":\"whiteoff\"}";
  }
  else if (cmd == "redon")
  {
    robot.turnOnRedLed();
    response = "{\"status\":\"OK\",\"action\":\"redon\"}";
  }
  else if (cmd == "redoff")
  {
    robot.turnOffRedLed();
    response = "{\"status\":\"OK\",\"action\":\"redoff\"}";
  }
  else if (cmd == "allledson")
  {
    robot.turnOnAllLeds();
    response = "{\"status\":\"OK\",\"action\":\"allledson\"}";
  }
  else if (cmd == "allledsoff")
  {
    robot.turnOffAllLeds();
    response = "{\"status\":\"OK\",\"action\":\"allledsoff\"}";
  }
  else if (cmd == "blinkwhite")
  {
    // interval>0, duty in [0..100], times>=0
    if (interval <= 0)
      response = "{\"status\":\"error\",\"message\":\"interval must be positive\"}";
    else if (duty < 0.f || duty > 100.f)
      response = "{\"status\":\"error\",\"message\":\"duty must be 0..100\"}";
    else if (times < 0)
      response = "{\"status\":\"error\",\"message\":\"times must be >=0\"}";
    else
    {
      robot.blinkWhiteLed(interval, duty, times);
      response = "{\"status\":\"OK\",\"action\":\"blinkwhite\"}";
    }
  }
  else if (cmd == "blinkred")
  {
    if (interval <= 0)
      response = "{\"status\":\"error\",\"message\":\"interval must be positive\"}";
    else if (duty < 0.f || duty > 100.f)
      response = "{\"status\":\"error\",\"message\":\"duty must be 0..100\"}";
    else if (times < 0)
      response = "{\"status\":\"error\",\"message\":\"times must be >=0\"}";
    else
    {
      robot.blinkRedLed(interval, duty, times);
      response = "{\"status\":\"OK\",\"action\":\"blinkred\"}";
    }
  }
  else
  {
    // Unknown command
    response = "{\"status\":\"error\",\"message\":\"Unknown command\"}";
  }

  ESP_LOGI(TAG, "handleCommand => %s", response.c_str());
  return response;
}

// Validate speed or duration
bool HttpController::checkSpeedRange(int speed)
{
  return (speed == -1) || (speed >= 0 && speed <= 100);
}
bool HttpController::checkDurationRange(int durationMs)
{
  return (durationMs == -1) || (durationMs >= 0 && durationMs <= 2000);
}
