#include "SolenoidKicker.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Tag for logging
static const char* TAG = "SolenoidKicker";

SolenoidKicker::SolenoidKicker(gpio_num_t gatePin, uint32_t defaultKickMs, int capSensePin)
    : gatePin(gatePin),
      defaultKickMs(defaultKickMs),
      capSensePin(capSensePin)
{
}

esp_err_t SolenoidKicker::begin()
{
    ESP_LOGI(TAG, "Initializing SolenoidKicker on pin %d...", gatePin);

    // Configure the gate pin as a push-pull output
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << gatePin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.intr_type    = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        // ESP_LOGE(TAG, "Failed to config GPIO for solenoid kicker: %s", esp_err_to_name(err));
        return err;
    }

    // Ensure gate is initially LOW (MOSFET off)
    gpio_set_level(gatePin, 0);

    // If using capSensePin, you could configure ADC here (not shown).
    // For example, using ESP-IDF’s ADC driver:
    //   - adc1_config_width(ADC_WIDTH_BIT_12);
    //   - adc1_config_channel_atten((adc1_channel_t)_capSensePin, ADC_ATTEN_DB_11);
    // The details depend on your hardware and IDF version.

    return ESP_OK;
}

void SolenoidKicker::kick(int durationMs)
{
    // If user passed 0, use the default
    if (durationMs == -1) {
        durationMs = defaultKickMs;
    }

    // Basic safety check
    if (durationMs > 2000) {
        // ESP_LOGW(TAG, "Kick duration seems too long (%d ms). Clamping to 2000 ms...", durationMs);
        durationMs = 2000; // arbitrary clamp
    }

    // ESP_LOGI(TAG, "Firing solenoid for %d ms on pin %d", durationMs, _gatePin);

    // Turn on MOSFET (drive gate HIGH)
    gpio_set_level(gatePin, 1);

    // Wait the specified duration
    vTaskDelay(pdMS_TO_TICKS(durationMs));

    // Turn off MOSFET (drive gate LOW)
    gpio_set_level(gatePin, 0);
}

void SolenoidKicker::setDefaultKickDuration(uint32_t durationMs)
{
    defaultKickMs = durationMs;
}

uint32_t SolenoidKicker::getDefaultKickDuration() const
{
    return defaultKickMs;
}

float SolenoidKicker::readCapVoltage()
{
    if (capSensePin < 0) {
        // Not configured for sensing
        return -1.0f;
    }

    // Example ADC read logic (for ADC1 channel). Actual code depends on your pin->ADC channel mapping:
    //   int raw = adc1_get_raw((adc1_channel_t)_capSensePin);
    //   // Convert raw reading to voltage
    //   // e.g. if 12-bit ADC, raw can be 0-4095, then scale to your reference. For attenuation DB_11 (~3.3v to ~3.6v range):
    //   float voltage = (raw / 4095.0f) * 3.6f; // or 3.3f, etc. This is a simplified example.
    //
    //   // If you have a voltage divider from the capacitor to the ADC, scale accordingly.
    //   // e.g. if you used a /4 divider for 12V max => multiply the raw reading by 4.
    //
    //   return voltage * (divider ratio);

    // For now, just a placeholder:
    return -1.0f; // Not implemented
}
