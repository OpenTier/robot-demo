#include "LedTelltale.h"
#include "esp_log.h"

static const char* TAG = "LedTelltale";

// A typical update rate for blinking logic. 10 ms = 100 updates/second.
static const TickType_t LED_TASK_DELAY_MS = 10;  

LedTelltale::LedTelltale(gpio_num_t whitePin, gpio_num_t redPin)
: whitePin_(whitePin),
  redPin_(redPin),
  taskHandle_(nullptr)
{
    // Initialize BlinkState defaults
    whiteBlink_.enabled         = false;
    whiteBlink_.intervalMs      = 1000;
    whiteBlink_.dutyCyclePercent= 50.0f;
    whiteBlink_.times           = 0;
    whiteBlink_.cyclesDone      = 0;
    whiteBlink_.isOn            = false;
    whiteBlink_.msInCycle       = 0;

    redBlink_.enabled           = false;
    redBlink_.intervalMs        = 1000;
    redBlink_.dutyCyclePercent  = 50.0f;
    redBlink_.times             = 0;
    redBlink_.cyclesDone        = 0;
    redBlink_.isOn              = false;
    redBlink_.msInCycle         = 0;
}

esp_err_t LedTelltale::init()
{
    // 1. Configure pins as outputs
    gpio_config_t io_conf = {};
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    io_conf.mode         = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << whitePin_) | (1ULL << redPin_);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LED pins: %s", esp_err_to_name(err));
        return err;
    }

    // Default off
    setPin(whitePin_, false);
    setPin(redPin_, false);

    // 2. Create the background task
    BaseType_t res = xTaskCreate(
        ledTaskFunc,
        "led_task",
        2048,      // stack size
        this,      // parameter
        5,         // priority (medium)
        &taskHandle_);

    if (res != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ledTaskFunc");
        return ESP_FAIL;
    }

    return ESP_OK;
}

// --- Basic On/Off ------------------------------------------------

void LedTelltale::turnOnWhite()
{
    // Optionally disable blink if we want direct control
    whiteBlink_.enabled = false;
    setPin(whitePin_, true);
}

void LedTelltale::turnOffWhite()
{
    whiteBlink_.enabled = false;
    setPin(whitePin_, false);
}

void LedTelltale::turnOnRed()
{
    redBlink_.enabled = false;
    setPin(redPin_, true);
}

void LedTelltale::turnOffRed()
{
    redBlink_.enabled = false;
    setPin(redPin_, false);
}

void LedTelltale::turnOnAll()
{
    whiteBlink_.enabled = false;
    redBlink_.enabled   = false;
    setPin(whitePin_, true);
    setPin(redPin_, true);
}

void LedTelltale::turnOffAll()
{
    whiteBlink_.enabled = false;
    redBlink_.enabled   = false;
    setPin(whitePin_, false);
    setPin(redPin_, false);
}

// --- Non-blocking blink calls -------------------------------------

void LedTelltale::blinkWhite(uint32_t intervalMs, float dutyCyclePercent, uint32_t times)
{
    if (dutyCyclePercent < 0.0f) dutyCyclePercent = 0.0f;
    if (dutyCyclePercent > 100.0f) dutyCyclePercent = 100.0f;

    whiteBlink_.enabled         = true;
    whiteBlink_.intervalMs      = intervalMs;
    whiteBlink_.dutyCyclePercent= dutyCyclePercent;
    whiteBlink_.times           = times;
    whiteBlink_.cyclesDone      = 0;
    whiteBlink_.msInCycle       = 0;
    whiteBlink_.isOn            = false;
}

void LedTelltale::blinkRed(uint32_t intervalMs, float dutyCyclePercent, uint32_t times)
{
    if (dutyCyclePercent < 0.0f) dutyCyclePercent = 0.0f;
    if (dutyCyclePercent > 100.0f) dutyCyclePercent = 100.0f;

    redBlink_.enabled         = true;
    redBlink_.intervalMs      = intervalMs;
    redBlink_.dutyCyclePercent= dutyCyclePercent;
    redBlink_.times           = times;
    redBlink_.cyclesDone      = 0;
    redBlink_.msInCycle       = 0;
    redBlink_.isOn            = false;
}

// --- Internal FreeRTOS task logic ---------------------------------

void LedTelltale::ledTaskFunc(void* pvParam)
{
    auto* that = static_cast<LedTelltale*>(pvParam);
    that->taskLoop(); // calls the member function
}

void LedTelltale::taskLoop()
{
    ESP_LOGI(TAG, "LED Task started.");

    const TickType_t delayTicks = pdMS_TO_TICKS(LED_TASK_DELAY_MS);

    while (true) {
        // 1. Update white LED if blinking
        if (whiteBlink_.enabled) {
            // Increment msInCycle by LED_TASK_DELAY_MS
            whiteBlink_.msInCycle += LED_TASK_DELAY_MS;
            // Compute onTime in ms
            float onTimeF = (whiteBlink_.intervalMs * (whiteBlink_.dutyCyclePercent / 100.0f));
            uint32_t onTime = (uint32_t)onTimeF;

            if (whiteBlink_.msInCycle <= onTime) {
                // Turn on
                setPin(whitePin_, true);
                whiteBlink_.isOn = true;
            } else {
                // Turn off
                setPin(whitePin_, false);
                whiteBlink_.isOn = false;
            }

            // If the cycle is complete
            if (whiteBlink_.msInCycle >= whiteBlink_.intervalMs) {
                whiteBlink_.msInCycle = 0;
                // If times != 0, check if we've completed all cycles
                if (whiteBlink_.times != 0) {
                    whiteBlink_.cyclesDone++;
                    if (whiteBlink_.cyclesDone >= whiteBlink_.times) {
                        // Done blinking
                        whiteBlink_.enabled = false;
                        setPin(whitePin_, false);
                    }
                }
            }
        }

        // 2. Update red LED if blinking
        if (redBlink_.enabled) {
            redBlink_.msInCycle += LED_TASK_DELAY_MS;
            float onTimeF = (redBlink_.intervalMs * (redBlink_.dutyCyclePercent / 100.0f));
            uint32_t onTime = (uint32_t)onTimeF;

            if (redBlink_.msInCycle <= onTime) {
                setPin(redPin_, true);
                redBlink_.isOn = true;
            } else {
                setPin(redPin_, false);
                redBlink_.isOn = false;
            }

            if (redBlink_.msInCycle >= redBlink_.intervalMs) {
                redBlink_.msInCycle = 0;
                if (redBlink_.times != 0) {
                    redBlink_.cyclesDone++;
                    if (redBlink_.cyclesDone >= redBlink_.times) {
                        redBlink_.enabled = false;
                        setPin(redPin_, false);
                    }
                }
            }
        }

        vTaskDelay(delayTicks);
    }
}

// Helper to set pin level
void LedTelltale::setPin(gpio_num_t pin, bool on)
{
    gpio_set_level(pin, on ? 1 : 0);
}
