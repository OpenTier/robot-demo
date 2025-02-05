#ifndef WIFI_COM_H
#define WIFI_COM_H

#include "esp_err.h"

/**
 * @brief Initialize the Wi-Fi connection.
 */
esp_err_t WifiComInit();

/**
 * @brief Periodically check and maintain the Wi-Fi connection (reconnect if needed).
 */
void WifiComTick();

#endif // WIFI_COM_H
