#pragma once

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief A class to control two LEDs (white and red) non-blocking.
 *
 *        We create an internal FreeRTOS task that handles blinking,
 *        so calls like blinkWhite() do not block the caller.
 */
class LedTelltale
{
public:
    /**
     * @brief Construct a new LedTelltale object.
     *
     * @param whitePin GPIO pin for the white LED.
     * @param redPin   GPIO pin for the red LED.
     */
    LedTelltale(gpio_num_t whitePin, gpio_num_t redPin);

    /**
     * @brief Initialize the GPIO pins and create the background task.
     *
     * @return esp_err_t ESP_OK on success, error code otherwise.
     */
    esp_err_t init();

    // --- Basic on/off control ---
    void turnOnWhite();
    void turnOffWhite();
    void turnOnRed();
    void turnOffRed();
    void turnOnAll();
    void turnOffAll();

    /**
     * @brief Non-blocking blink for the white LED.
     *
     * @param intervalMs Total period (in ms) for one blink cycle (on + off).
     * @param dutyCyclePercent How long (percentage) the LED stays ON in one cycle (0..100).
     * @param times Number of total blink cycles. If 0, blink indefinitely.
     */
    void blinkWhite(uint32_t intervalMs, float dutyCyclePercent = 50.0f, uint32_t times = 0);

    /**
     * @brief Non-blocking blink for the red LED.
     *
     * @param intervalMs Total period (in ms) for one blink cycle (on + off).
     * @param dutyCyclePercent How long (percentage) the LED stays ON in one cycle (0..100).
     * @param times Number of total blink cycles. If 0, blink indefinitely.
     */
    void blinkRed(uint32_t intervalMs, float dutyCyclePercent = 50.0f, uint32_t times = 0);

private:
    gpio_num_t whitePin_;
    gpio_num_t redPin_;

    // We'll store blinking parameters for each LED in a small struct
    struct BlinkState
    {
        bool     enabled;         // is blink active?
        uint32_t intervalMs;      // total cycle time
        float    dutyCyclePercent; // 0..100
        uint32_t times;           // how many cycles to blink (0 = infinite)
        uint32_t cyclesDone;      // how many completed so far

        // Runtime tracking
        bool     isOn;            // current LED state
        uint32_t msInCycle;       // how many ms have passed in current cycle
    };

    BlinkState whiteBlink_;
    BlinkState redBlink_;

    // Handle to the background task
    TaskHandle_t taskHandle_;

    /**
     * @brief Internal: sets the given LED pin (on/off).
     */
    void setPin(gpio_num_t pin, bool on);

    /**
     * @brief Internal: The static function that runs as a FreeRTOS task.
     *
     * @param pvParam 'this' pointer to LedTelltale.
     */
    static void ledTaskFunc(void* pvParam);

    /**
     * @brief Internal: The actual task loop that updates LED states every 10ms (example).
     */
    void taskLoop();
};
