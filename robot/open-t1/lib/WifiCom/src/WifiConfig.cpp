#include "WifiConfig.h"

#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char* TAG = "wifi_config";

// Adjust these defaults as needed
#ifndef DEFAULT_IP
#define DEFAULT_IP             "192.168.1.110"
#endif

#ifndef DEFAULT_PORT
#define DEFAULT_PORT           8888
#endif

#ifndef DEFAULT_WIFI_SSID
#define DEFAULT_WIFI_SSID      "OpenTier-Botball"
#endif

#ifndef DEFAULT_WIFI_PASSWORD
#define DEFAULT_WIFI_PASSWORD  "OpenTierBotball"
#endif

// If you want to always overwrite with defaults at compile time, define:
//    -DFORCE_DEFAULT_CONFIG
//
// e.g., add to your build config:
// [env:my_env]
// build_flags = -DFORCE_DEFAULT_CONFIG

// NVS Namespace
#define NVS_NAMESPACE "wifi_store"

typedef struct {
    char robot_ip[64];
    int  robot_port;
    char wifi_ssid[64];
    char wifi_password[64];
} WifiConfig;

// Global config struct in RAM
static WifiConfig wifi_config = {
    DEFAULT_IP,
    DEFAULT_PORT,
    DEFAULT_WIFI_SSID,
    DEFAULT_WIFI_PASSWORD
};

/**
 * @brief Save the wifi_config struct to NVS.
 */
static bool SaveWifiConfigToNvs()
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for writing: %s", esp_err_to_name(err));
        return false;
    }

    err |= nvs_set_str(handle, "robot_ip",      wifi_config.robot_ip);
    err |= nvs_set_i32(handle, "robot_port",    wifi_config.robot_port);
    err |= nvs_set_str(handle, "wifi_ssid",     wifi_config.wifi_ssid);
    err |= nvs_set_str(handle, "wifi_password", wifi_config.wifi_password);
    err |= nvs_commit(handle);

    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error writing Wi-Fi config to NVS: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "Wi-Fi config saved to NVS.");
    return true;
}

/**
 * @brief Load wifi_config from NVS. Returns true if valid data was found.
 */
static bool LoadWifiConfigFromNvs()
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No existing NVS config (err: %s)", esp_err_to_name(err));
        return false;
    }

    // Buffers
    char temp_ip[64]       = {0};
    char temp_ssid[64]     = {0};
    char temp_password[64] = {0};
    int  temp_port         = 0;

    size_t required_size   = 0;

    // robot_ip
    err = nvs_get_str(handle, "robot_ip", NULL, &required_size);
    if (err == ESP_OK && required_size < sizeof(temp_ip)) {
        nvs_get_str(handle, "robot_ip", temp_ip, &required_size);
    }

    // robot_port
    int32_t port_val = 0;
    if (nvs_get_i32(handle, "robot_port", &port_val) == ESP_OK) {
        temp_port = port_val;
    }

    // wifi_ssid
    required_size = 0;
    err = nvs_get_str(handle, "wifi_ssid", NULL, &required_size);
    if (err == ESP_OK && required_size < sizeof(temp_ssid)) {
        nvs_get_str(handle, "wifi_ssid", temp_ssid, &required_size);
    }

    // wifi_password
    required_size = 0;
    err = nvs_get_str(handle, "wifi_password", NULL, &required_size);
    if (err == ESP_OK && required_size < sizeof(temp_password)) {
        nvs_get_str(handle, "wifi_password", temp_password, &required_size);
    }

    nvs_close(handle);

    // Validate
    if (   temp_ip[0] != '\0'
        && temp_ssid[0] != '\0'
        && temp_password[0] != '\0'
        && temp_port != 0 )
    {
        strncpy(wifi_config.robot_ip, temp_ip, sizeof(wifi_config.robot_ip));
        wifi_config.robot_port = temp_port;
        strncpy(wifi_config.wifi_ssid, temp_ssid, sizeof(wifi_config.wifi_ssid));
        strncpy(wifi_config.wifi_password, temp_password, sizeof(wifi_config.wifi_password));
        ESP_LOGI(TAG, "Loaded Wi-Fi config from NVS");
        return true;
    }

    ESP_LOGW(TAG, "Wi-Fi configuration in NVS is invalid.");
    return false;
}

/**
 * @brief Initialize Wi-Fi config from NVS or defaults (and save).
 */
bool WifiConfigInit()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(err));
        return false;
    }

#ifdef FORCE_DEFAULT_CONFIG
    ESP_LOGW(TAG, "FORCE_DEFAULT_CONFIG is set. Overwriting with defaults.");
    return SaveWifiConfigToNvs();
#endif

    if (LoadWifiConfigFromNvs()) {
        ESP_LOGI(TAG, "Existing configuration loaded successfully.");
        return true;
    }

    ESP_LOGW(TAG, "No valid config found. Saving defaults.");
    return SaveWifiConfigToNvs();
}

// ------------- Accessors -------------

const char* GetIp() {
    return wifi_config.robot_ip;
}

int GetPort() {
    return wifi_config.robot_port;
}

const char* GetWifiSsid() {
    return wifi_config.wifi_ssid;
}

const char* GetWifiPassword() {
    return wifi_config.wifi_password;
}

// ------------- Mutators -------------

void SetRobotIp(const char* ip) {
    strncpy(wifi_config.robot_ip, ip, sizeof(wifi_config.robot_ip));
    SaveWifiConfigToNvs();
}

void SetRobotPort(int port) {
    wifi_config.robot_port = port;
    SaveWifiConfigToNvs();
}

void SetWifiSsid(const char* ssid) {
    strncpy(wifi_config.wifi_ssid, ssid, sizeof(wifi_config.wifi_ssid));
    SaveWifiConfigToNvs();
}

void SetWifiPassword(const char* password) {
    strncpy(wifi_config.wifi_password, password, sizeof(wifi_config.wifi_password));
    SaveWifiConfigToNvs();
}
