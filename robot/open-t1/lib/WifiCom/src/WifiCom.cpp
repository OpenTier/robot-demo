#include "WifiCom.h"
#include "WifiConfig.h"

#include <string.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h" // For IPSTR, IP2STR, and ESP-NETIF functions

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "wifi_com";

// Time (ms) to attempt reconnection before giving up
#define WIFI_RECONNECT_TIMEOUT_MS 5000

// Track connection status
static volatile bool s_wifi_connected = false;

// -------------------- Event Handler --------------------
static void WifiEventHandler(void *arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, "Wi-Fi STA connected.");
        s_wifi_connected = true;
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_connected = false;
        ESP_LOGW(TAG, "Wi-Fi disconnected.");
        // We'll reconnect in WifiComTick() or call esp_wifi_connect() here
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        s_wifi_connected = true;
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    } else {
        ESP_LOGW(TAG, "Unhandled event: %s:%d", event_base, event_id);
    }
}


// -------------------- Wi-Fi init -----------------------
static void WifiInitModule()
{
    // Initialize the underlying TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create default station
    esp_netif_create_default_wifi_sta();

    // init Wi-Fi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &WifiEventHandler,
        NULL,
        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &WifiEventHandler,
        NULL,
        NULL));

    // Configure station
    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid, GetWifiSsid(), sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, GetWifiPassword(), sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK; // Adjust if needed
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    // Start Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Wi-Fi station started.");

    // Wait for connection to be established
    uint32_t start_time = xTaskGetTickCount();
    while (!s_wifi_connected)
    {
        vTaskDelay(pdMS_TO_TICKS(100)); // Poll every 100ms
        if (((xTaskGetTickCount() - start_time) * portTICK_PERIOD_MS) > WIFI_RECONNECT_TIMEOUT_MS)
        {
            ESP_LOGE(TAG, "Wi-Fi connection timeout!");
            break;
        }
    }
    if (s_wifi_connected)
    {
        ESP_LOGI(TAG, "Wi-Fi connected successfully.");
    }
    else
    {
        ESP_LOGE(TAG, "Wi-Fi connection failed.");
    }
}

/**
 * @brief Attempt to reconnect if not connected.
 */
static void CheckReconnect()
{
    if (!s_wifi_connected)
    {
        ESP_LOGW(TAG, "Reconnecting to Wi-Fi...");
        uint32_t start_time = xTaskGetTickCount();
        esp_wifi_connect();

        // Poll for connection or timeout
        while (!s_wifi_connected)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            if ((xTaskGetTickCount() - start_time) * portTICK_PERIOD_MS > WIFI_RECONNECT_TIMEOUT_MS)
            {
                ESP_LOGE(TAG, "Wi-Fi reconnection timeout!");
                break;
            }
        }
    }
}

// -------------------------------------------------------
esp_err_t WifiComInit()
{
    ESP_LOGI(TAG, "Initializing Wi-Fi config...");

    if (!WifiConfigInit())
    {
        ESP_LOGE(TAG, "Failed to load Wi-Fi config from NVS");
        // Return a suitable error code
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Connecting to SSID: %s", GetWifiSsid());

    // Suppose WifiInitSta() returns `esp_err_t`.
    WifiInitModule();

    ESP_LOGI(TAG, "Wi-Fi connected! init complete.");
    return ESP_OK;
}

void WifiComTick()
{
    if (!s_wifi_connected)
    {
        CheckReconnect();
    }
}
