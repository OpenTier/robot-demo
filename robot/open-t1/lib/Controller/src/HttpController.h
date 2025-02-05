#pragma once

#include <string>
#include "esp_err.h"
#include "esp_http_server.h"
#include "IController.h"
#include "Robot.h"

/**
 * @brief A controller that handles plain HTTP requests to control the Robot.
 *
 * Usage:
 *  - GET /cmd?command=forward|backward|left|right|rotateLeft|rotateRight|stop|kick|battery|normalizedcontrol|control|...
 *    * speed=[0..100 or -1]
 *    * duration=[0..2000 or -1]
 *    * x,y,rot for control
 *
 * Examples:
 *   GET /cmd?command=forward&speed=50
 *   GET /cmd?command=kick&duration=1000
 *   GET /cmd?command=battery
 *   GET /cmd?command=normalizedcontrol&x=30&y=50&rot=-20
 *   GET /cmd?command=control&x=200&y=300&rot=1.57
 *
 * LED commands:
 *   GET /cmd?command=whiteon
 *   GET /cmd?command=whiteoff
 *   GET /cmd?command=redon
 *   GET /cmd?command=redoff
 *   GET /cmd?command=allledson
 *   GET /cmd?command=allledsoff
 *   GET /cmd?command=blinkwhite&interval=1000&duty=50&times=0
 *   GET /cmd?command=blinkred&interval=500&duty=80&times=5
 */
class HttpController : public IController
{
public:
    explicit HttpController(Robot &r);

    esp_err_t init() override;
    void update() override;

private:
    static esp_err_t cmdHandler(httpd_req_t *req);

    /**
     * @brief Process a command + optional parameters from the query string.
     *
     * @param command   e.g. "forward", "kick", "battery", "normalizedcontrol", "control", "whiteon", etc.
     * @param speed     integer or -1
     * @param duration  integer or -1
     * @param xVal      float: used by normalizedcontrol or control
     * @param yVal      float: used by normalizedcontrol or control
     * @param rotVal    float: used by normalizedcontrol or control
     * @param interval  used by blink
     * @param duty      used by blink
     * @param times     used by blink
     * @return JSON response string
     */
    std::string handleCommand(const std::string &command,
                              int speed, int duration,
                              float xVal, float yVal, float rotVal,
                              int interval, float duty, int times);

    // Validations
    bool checkSpeedRange(int speed);
    bool checkDurationRange(int durationMs);

private:
    Robot &robot;
    httpd_handle_t serverHandle; // HTTP server handle

    static const char *TAG;
};
