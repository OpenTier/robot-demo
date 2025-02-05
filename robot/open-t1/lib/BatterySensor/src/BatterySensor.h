#pragma once

// #include "driver/adc_common.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

/**
 * @brief BatterySensor - simple driver to read battery voltage from an analog pin using a resistor divider.
 */
class BatterySensor {
public:
    /**
     * @brief Construct the BatterySensor with the given ADC unit, channel, and scale factor (divider ratio).
     * @param adcUnit    Which ADC peripheral (ADC_UNIT_1 or ADC_UNIT_2).
     * @param channel    Which ADC channel (e.g. ADC_CHANNEL_6).
     * @param divider    Voltage divider ratio. E.g. 5.0 for a 5:1 divider.
     */
    BatterySensor(adc_unit_t adcUnit, adc_channel_t channel, float divider);

    /**
     * @brief Initialize the ADC channel in "oneshot" mode and (optionally) set up calibration if needed.
     */
    esp_err_t init();

    /**
     * @brief Read the battery voltage in volts.
     */
    float readVoltage() const;

private:
    adc_unit_t      adcUnit;
    adc_channel_t   adcChannel;
    float           divider;
    adc_oneshot_unit_handle_t adcHandle;
};
