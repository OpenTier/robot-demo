#pragma once

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <cctype>
#include <climits> // for INT_MIN, INT_MAX
#include <cmath>   // for strtof
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// For USB Serial-JTAG console
#include "esp_vfs_usb_serial_jtag.h"

#include "IController.h"
#include "Robot.h"

class SerialController : public IController
{
public:
    explicit SerialController(Robot &r)
        : robot(r)
    {
    }

    /**
     * @brief Initialize to read/write via the default USB Serial-JTAG console.
     *
     * @return esp_err_t - Always returns ESP_OK unless your IDF is missing the USB Serial-JTAG VFS.
     */
    esp_err_t init() override
    {
        // Configure line endings and buffering for USB Serial-JTAG
        esp_vfs_dev_usb_serial_jtag_set_rx_line_endings(ESP_LINE_ENDINGS_CRLF);
        esp_vfs_dev_usb_serial_jtag_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

        setvbuf(stdin, nullptr, _IONBF, 0);
        setvbuf(stdout, nullptr, _IONBF, 0);

        printf("SerialController (USB Serial-JTAG) initialized. Type 'help' for commands.\r\n> ");
        return ESP_OK;
    }

    /**
     * @brief Read characters from stdin (non-blocking) and parse them if we get a full line.
     */
    void update() override
    {
        while (true)
        {
            int c = fgetc(stdin);
            if (c == EOF)
            {
                // No more chars right now
                break;
            }
            else if (c == '\r')
            {
                // Ignore carriage returns
            }
            else if (c == '\n')
            {
                // We got a full line, parse it
                parseLine(inputBuffer);
                inputBuffer.clear();
                // Print shell prompt again
                printf("> ");
            }
            else
            {
                // Accumulate the character
                inputBuffer.push_back(static_cast<char>(c));
            }
        }
    }

private:
    void parseLine(const std::string &line)
    {
        if (line.empty())
        {
            return;
        }

        // Tokenize by spaces
        std::vector<std::string> tokens;
        {
            size_t start = 0;
            while (true)
            {
                size_t idx = line.find(' ', start);
                if (idx == std::string::npos)
                {
                    tokens.push_back(line.substr(start));
                    break;
                }
                else
                {
                    tokens.push_back(line.substr(start, idx - start));
                    start = idx + 1;
                }
            }
        }

        // Lowercase the first token for the command
        std::string cmd = tokens[0];
        for (auto &ch : cmd)
        {
            ch = static_cast<char>(tolower(static_cast<unsigned char>(ch)));
        }

        // Dispatch
        if (cmd == "help")
        {
            printHelp();
        }
        else if (cmd == "forward")
        {
            handleMovementCommand(&Robot::moveForward, tokens);
        }
        else if (cmd == "backward")
        {
            handleMovementCommand(&Robot::moveBackward, tokens);
        }
        else if (cmd == "left")
        {
            handleMovementCommand(&Robot::moveLeft, tokens);
        }
        else if (cmd == "right")
        {
            handleMovementCommand(&Robot::moveRight, tokens);
        }
        else if (cmd == "rotateleft")
        {
            handleMovementCommand(&Robot::rotateLeft, tokens);
        }
        else if (cmd == "rotateright")
        {
            handleMovementCommand(&Robot::rotateRight, tokens);
        }
        else if (cmd == "stop")
        {
            robot.stop();
        }
        else if (cmd == "kick")
        {
            handleKickCommand(tokens);
        }
        else if (cmd == "battery")
        {
            float voltage = robot.getBatteryVoltage();
            printf("Battery Voltage: %.2f V\r\n", voltage);
        }
        else if (cmd == "normalizedcontrol")
        {
            // normalizedcontrol <x> <y> <rotation>
            if (tokens.size() < 4)
            {
                printf("Usage: normalizedcontrol <xPercent> <yPercent> <rotationPercent>\r\n");
                return;
            }

            int xVal, yVal, rotVal;
            if (!parseInt(tokens[1], xVal) ||
                !parseInt(tokens[2], yVal) ||
                !parseInt(tokens[3], rotVal))
            {
                printf("Error: parameters must be integers.\r\n");
                return;
            }
            robot.controlVelocityNormalized(xVal, yVal, rotVal);
        }
        else if (cmd == "control")
        {
            // control <x_mm_s> <y_mm_s> <rot_rad_s>
            if (tokens.size() < 4)
            {
                printf("Usage: control <x_mm_s> <y_mm_s> <rot_rad_s>\r\n");
                return;
            }

            float xVal, yVal, rotVal;
            if (!parseFloat(tokens[1], xVal) ||
                !parseFloat(tokens[2], yVal) ||
                !parseFloat(tokens[3], rotVal))
            {
                printf("Error: parameters must be floats.\r\n");
                return;
            }
            robot.controlVelocity(xVal, yVal, rotVal);
        }
        else if (cmd == "whiteon")
        {
            robot.turnOnWhiteLed();
        }
        else if (cmd == "whiteoff")
        {
            robot.turnOffWhiteLed();
        }
        else if (cmd == "redon")
        {
            robot.turnOnRedLed();
        }
        else if (cmd == "redoff")
        {
            robot.turnOffRedLed();
        }
        else if (cmd == "allledson")
        {
            robot.turnOnAllLeds();
        }
        else if (cmd == "allledsoff")
        {
            robot.turnOffAllLeds();
        }
        else if (cmd == "blinkwhite")
        {
            // Usage: blinkwhite <intervalMs> [duty=50] [times=0]
            // e.g. blinkwhite 1000 60 5
            handleBlinkWhite(tokens);
        }
        else if (cmd == "blinkred")
        {
            // Usage: blinkred <intervalMs> [duty=50] [times=0]
            handleBlinkRed(tokens);
        }
        else
        {
            printf("Unknown command. Type 'help' for usage.\r\n");
        }
    }

    void printHelp()
    {
        printf("Available commands:\r\n");
        printf("  help\r\n");
        printf("\r\n");
        printf("Movements Commands:\r\n");
        printf("  forward [0..100]\r\n");
        printf("  backward [0..100]\r\n");
        printf("  left [0..100]\r\n");
        printf("  right [0..100]\r\n");
        printf("  rotateleft [0..100]\r\n");
        printf("  rotateright [0..100]\r\n");
        printf("  stop\r\n");
        printf("  normalizedcontrol <xPercent> <yPercent> <rotationPercent>\r\n");
        printf("  control <x_mm_s> <y_mm_s> <rot_rad_s>\r\n");
        printf("  kick [0..2000]  (ms)\r\n");
        printf("\r\n");
        printf("Health Commands:\r\n");
        printf("  battery\r\n");
        printf("\r\n");
        printf("LED Commands:\r\n");
        printf("  whiteon / whiteoff\r\n");
        printf("  redon / redoff\r\n");
        printf("  allledson / allledsoff\r\n");
        printf("  blinkwhite <intervalMs> [dutyCycle=50] [times=0]\r\n");
        printf("  blinkred   <intervalMs> [dutyCycle=50] [times=0]\r\n");
    }

    void handleMovementCommand(void (Robot::*func)(int), const std::vector<std::string> &tokens)
    {
        if (tokens.size() > 1)
        {
            int speed;
            if (!parseInt(tokens[1], speed))
            {
                printf("Error: Invalid speed (must be integer 0..100)\r\n");
                return;
            }
            if (speed < 0 || speed > 100)
            {
                printf("Error: Speed must be 0..100.\r\n");
                return;
            }
            (robot.*func)(speed);
        }
        else
        {
            // No speed => pass -1 so Robot can use default
            (robot.*func)(-1);
        }
    }

    void handleKickCommand(const std::vector<std::string> &tokens)
    {
        if (tokens.size() > 1)
        {
            int duration;
            if (!parseInt(tokens[1], duration))
            {
                printf("Error: Invalid duration (0..3000ms)\r\n");
                return;
            }
            if (duration < 0 || duration > 2000)
            {
                printf("Error: Kick duration must be 0..2000.\r\n");
                return;
            }
            robot.kick(static_cast<uint32_t>(duration));
        }
        else
        {
            robot.kick(); // default
        }
    }

    void handleBlinkWhite(const std::vector<std::string> &tokens)
    {
        // blinkwhite <intervalMs> [duty=50] [times=0]
        if (tokens.size() < 2)
        {
            printf("Usage: blinkwhite <intervalMs> [dutyCycle=50] [times=0]\r\n");
            return;
        }
        int intervalMs;
        if (!parseInt(tokens[1], intervalMs) || intervalMs <= 0)
        {
            printf("Error: intervalMs must be a positive integer.\r\n");
            return;
        }

        float dutyCycle = 50.0f; // default
        if (tokens.size() >= 3)
        {
            float tmp;
            if (!parseFloat(tokens[2], tmp) || tmp < 0.0f || tmp > 100.0f)
            {
                printf("Error: dutyCycle must be 0..100.\r\n");
                return;
            }
            dutyCycle = tmp;
        }

        int times = 0; // default infinite
        if (tokens.size() >= 4)
        {
            int tmpInt;
            if (!parseInt(tokens[3], tmpInt) || tmpInt < 0)
            {
                printf("Error: times must be a non-negative integer.\r\n");
                return;
            }
            times = tmpInt;
        }

        robot.blinkWhiteLed(intervalMs, dutyCycle, times);
    }

    void handleBlinkRed(const std::vector<std::string> &tokens)
    {
        // blinkred <intervalMs> [duty=50] [times=0]
        if (tokens.size() < 2)
        {
            printf("Usage: blinkred <intervalMs> [dutyCycle=50] [times=0]\r\n");
            return;
        }
        int intervalMs;
        if (!parseInt(tokens[1], intervalMs) || intervalMs <= 0)
        {
            printf("Error: intervalMs must be a positive integer.\r\n");
            return;
        }

        float dutyCycle = 50.0f; // default
        if (tokens.size() >= 3)
        {
            float tmp;
            if (!parseFloat(tokens[2], tmp) || tmp < 0.0f || tmp > 100.0f)
            {
                printf("Error: dutyCycle must be 0..100.\r\n");
                return;
            }
            dutyCycle = tmp;
        }

        int times = 0; // default infinite
        if (tokens.size() >= 4)
        {
            int tmpInt;
            if (!parseInt(tokens[3], tmpInt) || tmpInt < 0)
            {
                printf("Error: times must be a non-negative integer.\r\n");
                return;
            }
            times = tmpInt;
        }

        robot.blinkRedLed(intervalMs, dutyCycle, times);
    }

    bool parseInt(const std::string &text, int &outValue)
    {
        if (text.empty())
        {
            return false;
        }
        char *endPtr = nullptr;
        long val = std::strtol(text.c_str(), &endPtr, 10);
        if (*endPtr != '\0')
        {
            // Some non-digit chars
            return false;
        }
        if (val < INT_MIN || val > INT_MAX)
        {
            return false;
        }
        outValue = static_cast<int>(val);
        return true;
    }

    bool parseFloat(const std::string &text, float &outValue)
    {
        if (text.empty())
        {
            return false;
        }
        char *endPtr = nullptr;
        float val = strtof(text.c_str(), &endPtr);
        if (*endPtr != '\0')
        {
            // Some non-digit chars
            return false;
        }
        outValue = val;
        return true;
    }

private:
    Robot &robot;
    std::string inputBuffer;
};
