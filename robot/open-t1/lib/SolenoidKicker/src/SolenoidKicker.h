#pragma once

#include "driver/gpio.h"
#include "esp_err.h"
#include <stdint.h>

/**
 * @brief SolenoidKicker - A simple class to handle a capacitor-discharge solenoid (kicker) on 12V,
 * controlled by an ESP32 GPIO pin driving a MOSFET or IGBT.
 *
 * Usage Steps:
 *  1. Instantiate with a chosen GPIO pin for the gate (low-side switch).
 *  2. Call begin() to configure the pin as an output.
 *  3. Use kick() to discharge the capacitor through the solenoid for the specified duration.
 *  4. Optionally, monitor capacitor voltage with an ADC input if you want to check readiness.
 */
class SolenoidKicker
{
public:
    /**
     * @brief Construct the SolenoidKicker object with the specified gate pin and default kick duration.
     * @param gatePin The ESP32 GPIO used to drive the MOSFET’s gate.
     * @param defaultKickMs Default duration (milliseconds) for the solenoid pulse.
     * @param capSensePin (Optional) ADC-capable pin to measure capacitor voltage.
     *                    Pass -1 if you don’t need this feature.
     */
    SolenoidKicker(gpio_num_t gatePin, uint32_t defaultKickMs = 50, int capSensePin = -1);

    /**
     * @brief Initialize the GPIO pin for output. Must be called before using kick().
     * @return ESP_OK on success, or an error code from gpio_config on failure.
     */
    esp_err_t begin();

    /**
     * @brief Fire the solenoid for the specified duration (or default if none provided).
     * @param durationMs Duration in milliseconds. If 0, uses _defaultKickMs.
     */
    void kick(int durationMs = -1);

    /**
     * @brief Set the default pulse length in milliseconds.
     * @param durationMs The new default duration (e.g. 50 ms).
     */
    void setDefaultKickDuration(uint32_t durationMs);

    /**
     * @brief Get the current default pulse length in milliseconds.
     */
    uint32_t getDefaultKickDuration() const;

    /**
     * @brief (Optional) Read the capacitor voltage via ADC (if capSensePin != -1).
     * @return Voltage in volts, or -1.0 if not configured for sensing.
     *
     * NOTE: This requires setting up the ADC in menuconfig or using ADC APIs from ESP-IDF.
     * This function is just a placeholder to show how you might read the capacitor level.
     */
    float readCapVoltage();

private:
    gpio_num_t gatePin;     ///< The MOSFET/IGBT gate GPIO pin
    uint32_t defaultKickMs; ///< Default solenoid “kick” pulse duration (ms)
    int capSensePin;        ///< ADC-capable pin for capacitor voltage monitoring (-1 if unused)
};
