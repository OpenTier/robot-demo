#include "BatterySensor.h"
#include "esp_log.h"

static const char *TAG = "BatterySensor";
static const float DIODE_DROP = 0.35f; // typical diode drop for a Schottky diode

// For ESP32-S3, recommended config is 12-bit resolution, up to ~4095 raw.
BatterySensor::BatterySensor(adc_unit_t adcUnit, adc_channel_t channel, float divider)
    : adcUnit(adcUnit), adcChannel(channel), divider(divider), adcHandle(nullptr)
{
}

esp_err_t BatterySensor::init()
{
    // 1) Configure the ADC unit
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = adcUnit,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
        // New field in ESP-IDF 5/6:
        // .clk_src = ADC_ONESHOT_CLK_SRC_DEFAULT
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &adcHandle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to init ADC unit: %s", esp_err_to_name(err));
        return err;
    }

    // 2) Configure the channel
    // Note that the structure order changed in newer IDF versions
    // Also, ADC_ATTEN_DB_11 is deprecated; you can use ADC_ATTEN_DB_12 now.
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,   // was DB_11 in older docs
        .bitwidth = ADC_BITWIDTH_12 // 12-bit resolution
    };
    err = adc_oneshot_config_channel(adcHandle, adcChannel, &config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to config ADC channel: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

float BatterySensor::readVoltage() const
{
    if (!adcHandle)
    {
        ESP_LOGW(TAG, "ADC not initialized!");
        return 0.0f;
    }

    int raw = 0;
    esp_err_t err = adc_oneshot_read(adcHandle, adcChannel, &raw);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "ADC read error: %s", esp_err_to_name(err));
        return 0.0f;
    }

    // The default raw range is 0-4095 for 12 bits.
    // With attenuation = 11dB, the input range can be up to ~3.6 V (esp32-s3 docs).
    // For a typical 3.3 V system:
    float measuredVoltage = (float)raw / 4095.0f * 3.3f;
    // Scale up by the resistor divider factor
    float batteryVoltage = measuredVoltage * divider + DIODE_DROP; // add 0.35V for the diode drop

    return batteryVoltage;
}
