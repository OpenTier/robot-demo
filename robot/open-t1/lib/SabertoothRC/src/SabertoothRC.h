#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/mcpwm_prelude.h"

class SabertoothRC {
public:
    /**
     * @brief Construct a SabertoothRC driver instance.
     * @param gpioS1 GPIO number for channel S1 (motor 1 control).
     * @param gpioS2 GPIO number for channel S2 (motor 2 control).
     */
    SabertoothRC(int gpioS1, int gpioS2);

    /**
     * @brief Initialize the MCPWM channels for RC signal output.
     * @param frequency PWM frequency in Hz (default 50Hz, typical for RC servo signals).
     * @return ESP_OK on success, or an error code from ESP-IDF on failure.
     *
     * This configures a MCPWM timer and operator, and sets up two PWM outputs on the given GPIOs.
     * It uses a 1 MHz time base (1 µs resolution) and a period corresponding to the given frequency.
     */
    esp_err_t begin(int group_id, uint32_t frequency = 50);

    /**
     * @brief Set the speed (output pulse width) for motor channel 1 (S1).
     * @param percent Speed as a percentage from -100.0 to 100.0 (negative for reverse, positive for forward).
     * @return ESP_OK on success, or an ESP-IDF error code.
     *
     * -100% will correspond to minimum pulse width (e.g. ~1000 µs), 
     * +100% to maximum pulse width (e.g. ~2000 µs), and 0% to neutral (~1500 µs).
     */
    esp_err_t setMotor1(float percent);

    /**
     * @brief Set the speed (output pulse width) for motor channel 2 (S2).
     * @param percent Speed as a percentage from -100.0 to 100.0.
     * @return ESP_OK on success, or an ESP-IDF error code.
     */
    esp_err_t setMotor2(float percent);

    /**
     * @brief (Optional) Set speed for a given motor channel index.
     * @param motor Index of motor (1 or 2).
     * @param percent Speed percentage (-100.0 to 100.0).
     * @return ESP_OK on success, or ESP_ERR_INVALID_ARG if motor index is out of range.
     */
    esp_err_t setMotorSpeed(int motor, float percent);

private:
    int _gpioS1;
    int _gpioS2;
    mcpwm_timer_handle_t    _timer       = nullptr;
    mcpwm_oper_handle_t     _operator    = nullptr;
    mcpwm_cmpr_handle_t     _comparator1 = nullptr;
    mcpwm_cmpr_handle_t     _comparator2 = nullptr;
    mcpwm_gen_handle_t      _generator1  = nullptr;
    mcpwm_gen_handle_t      _generator2  = nullptr;
    uint32_t                _frequency   = 50;  // Hz

    // Helper to convert percentage (-100 to 100) to comparator value (ticks) for 1us resolution
    uint32_t percentToTicks(float percent) const;
};
