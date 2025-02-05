#include "SabertoothRC.h"

SabertoothRC::SabertoothRC(int gpioS1, int gpioS2)
    : _gpioS1(gpioS1), _gpioS2(gpioS2) {
    // Constructor - just store GPIO pins, actual setup done in begin()
}

esp_err_t SabertoothRC::begin(int group_id, uint32_t frequency) {
    _frequency = frequency;
    // Calculate timer period in ticks (1 tick = 1 µs) for the given frequency
    // Using 1 MHz resolution (1us per tick)
    uint32_t resolution_hz = 1000000;  // 1 MHz
    // Period (ticks) = 1e6 / frequency (rounded to nearest integer)
    double period_f = 1000000.0 / static_cast<double>(_frequency);
    uint32_t period_ticks = static_cast<uint32_t>(period_f + 0.5);  // round to nearest

    // 1. Create MCPWM timer
    mcpwm_timer_config_t timer_config = {};
    timer_config.group_id = 0;                             // use MCPWM group 0
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;    // use default clock source
    timer_config.resolution_hz = resolution_hz;
    timer_config.period_ticks = period_ticks;
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;   // count up mode
    esp_err_t err = mcpwm_new_timer(&timer_config, &_timer);
    if (err != ESP_OK) {
        return err;
    }

    // 2. Create MCPWM operator (generator unit)
    mcpwm_operator_config_t operator_config = {};
    operator_config.group_id = 0;  // operator must be in the same group as the timer
    err = mcpwm_new_operator(&operator_config, &_operator);
    if (err != ESP_OK) {
        return err;
    }

    // 3. Connect operator to timer
    err = mcpwm_operator_connect_timer(_operator, _timer);
    if (err != ESP_OK) {
        return err;
    }

    // 4. Create comparators for each channel (S1, S2)
    mcpwm_comparator_config_t comparator_config = {};
    comparator_config.flags.update_cmp_on_tez = true;  // update compare at timer start (TEZ = Timer Event Zero)
    err = mcpwm_new_comparator(_operator, &comparator_config, &_comparator1);
    if (err != ESP_OK) {
        return err;
    }
    err = mcpwm_new_comparator(_operator, &comparator_config, &_comparator2);
    if (err != ESP_OK) {
        return err;
    }

    // 5. Create generators for each channel and bind them to the specified GPIOs
    mcpwm_generator_config_t gen_config = {};
    gen_config.flags.invert_pwm = false;  // not inverted (default behavior)
    // Generator for channel S1:
    gen_config.gen_gpio_num = _gpioS1;
    err = mcpwm_new_generator(_operator, &gen_config, &_generator1);
    if (err != ESP_OK) {
        return err;
    }
    // Generator for channel S2:
    gen_config.gen_gpio_num = _gpioS2;
    err = mcpwm_new_generator(_operator, &gen_config, &_generator2);
    if (err != ESP_OK) {
        return err;
    }

    // 6. Set initial pulse width to neutral (50% between min and max pulse widths, i.e., ~1500 µs)
    uint32_t neutral_ticks = percentToTicks(0.0f);
    err = mcpwm_comparator_set_compare_value(_comparator1, neutral_ticks);
    if (err != ESP_OK) {
        return err;
    }
    err = mcpwm_comparator_set_compare_value(_comparator2, neutral_ticks);
    if (err != ESP_OK) {
        return err;
    }

    // 7. Configure generator actions:
    // On timer reset (start of each cycle), set outputs high, and on reaching comparator value, set outputs low.
    err = mcpwm_generator_set_action_on_timer_event(
            _generator1,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    if (err != ESP_OK) {
        return err;
    }
    err = mcpwm_generator_set_action_on_timer_event(
            _generator2,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    if (err != ESP_OK) {
        return err;
    }
    // Set each generator to drive LOW when its comparator is reached (end of pulse)
    err = mcpwm_generator_set_action_on_compare_event(
            _generator1,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, _comparator1, MCPWM_GEN_ACTION_LOW));
    if (err != ESP_OK) {
        return err;
    }
    err = mcpwm_generator_set_action_on_compare_event(
            _generator2,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, _comparator2, MCPWM_GEN_ACTION_LOW));
    if (err != ESP_OK) {
        return err;
    }

    // 8. Enable and start the timer (starts PWM signal generation)
    err = mcpwm_timer_enable(_timer);
    if (err != ESP_OK) {
        return err;
    }
    err = mcpwm_timer_start_stop(_timer, MCPWM_TIMER_START_NO_STOP);  // start indefinitely
    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

esp_err_t SabertoothRC::setMotor1(float percent) {
    // Update comparator for channel 1 (S1)
    if (_comparator1 == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    uint32_t compare_ticks = percentToTicks(percent);
    return mcpwm_comparator_set_compare_value(_comparator1, compare_ticks);
}

esp_err_t SabertoothRC::setMotor2(float percent) {
    // Update comparator for channel 2 (S2)
    if (_comparator2 == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    uint32_t compare_ticks = percentToTicks(percent);
    return mcpwm_comparator_set_compare_value(_comparator2, compare_ticks);
}

esp_err_t SabertoothRC::setMotorSpeed(int motor, float percent) {
    if (motor == 1) {
        return setMotor1(percent);
    } else if (motor == 2) {
        return setMotor2(percent);
    }
    return ESP_ERR_INVALID_ARG;  // invalid motor index
}

uint32_t SabertoothRC::percentToTicks(float percent) const {
    // Clamp the input percentage
    if (percent > 100.0f)   percent = 100.0f;
    if (percent < -100.0f)  percent = -100.0f;
    // Define min and max pulse widths (in microseconds) for 100% ranges
    const float min_us = 1000.0f;  // 1000 µs pulse = -100% (full reverse)
    const float max_us = 2000.0f;  // 2000 µs pulse = +100% (full forward)
    const float mid_us = (min_us + max_us) / 2.0f;      // 1500 µs center (neutral)
    const float half_range = (max_us - min_us) / 2.0f;  // 500 µs from center to min or max
    // Calculate pulse width corresponding to the percentage
    float pulse_us = mid_us + (percent / 100.0f) * half_range;
    // Convert to ticks (at 1 MHz resolution, 1 tick = 1 µs)
    return static_cast<uint32_t>(pulse_us + 0.5f);  // add 0.5 for rounding to nearest tick
}
