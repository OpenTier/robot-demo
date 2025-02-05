#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include <stdbool.h>

/**
 * @brief Initialize Wi-Fi config from NVS or defaults.
 *
 * @return true if successful, otherwise false
 */
bool WifiConfigInit();

/**
 * @brief Get the stored robot IP address
 */
const char* GetIp();

/**
 * @brief Get the stored robot port
 */
int GetPort();

/**
 * @brief Get the stored Wi-Fi SSID
 */
const char* GetWifiSsid();

/**
 * @brief Get the stored Wi-Fi password
 */
const char* GetWifiPassword();

// Setters that automatically save to flash
void SetRobotIp(const char* ip);
void SetRobotPort(int port);
void SetWifiSsid(const char* ssid);
void SetWifiPassword(const char* password);

#endif // WIFI_CONFIG_H
