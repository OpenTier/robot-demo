#include "Robot.h"
#include "esp_log.h"
#include <sstream>
#include <cctype>

static const char *Tag = "Robot";

Robot::Robot(int pwmLeftFront, int pwmLeftRear,
             int pwmRightFront, int pwmRightRear,
             gpio_num_t solenoidGatePin, int defaultKickMs,
             adc_unit_t batteryAdcUnit, adc_channel_t batteryAdcChannel, float batteryDivider,
             gpio_num_t whiteLedPin, gpio_num_t redLedPin,
             int setMaxSpeed)
    : leftController(pwmLeftFront, pwmLeftRear),
      rightController(pwmRightFront, pwmRightRear),
      kicker(solenoidGatePin, defaultKickMs, -1),
      battery(batteryAdcUnit, batteryAdcChannel, batteryDivider),
      drive(leftController, rightController, 14.0),
      ledTelltale(whiteLedPin, redLedPin),
      maxSpeed(setMaxSpeed)
{
}

esp_err_t Robot::init()
{
    esp_err_t ret = leftController.begin(0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(Tag, "Failed to initialize left controller: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(Tag, "Left controller initialized.");

    ret = rightController.begin(1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(Tag, "Failed to initialize right controller: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(Tag, "Right controller initialized.");

    ret = kicker.begin();
    if (ret != ESP_OK)
    {
        ESP_LOGE(Tag, "Failed to initialize kicker: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(Tag, "Kicker initialized.");

    ret = battery.init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(Tag, "Failed to initialize battery sensor: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(Tag, "Battery sensor initialized.");

    // Initialize the LedTelltale
    ret = ledTelltale.init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(Tag, "Failed to init LED Telltale: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(Tag, "LED Telltale initialized.");

    ESP_LOGI(Tag, "Robot initialization complete.");
    return ESP_OK;
}

void Robot::update()
{
    float volts = battery.readVoltage();
    ESP_LOGI(Tag, "Battery voltage: %.2f V", volts);
}

float Robot::getBatteryVoltage() const
{
    return battery.readVoltage();
}

void Robot::kick(int kickDuration)
{
    kicker.kick(kickDuration);
}

void Robot::moveForward(int speedPercent)
{
    // If user provided a negative or invalid speed, cap it to maxSpeed
    int spd = (speedPercent < 0 ? maxSpeed : speedPercent);
    // +X => Forward
    drive.controlVelocityNormalized(spd, 0, 0);
}

void Robot::moveBackward(int speedPercent)
{
    int spd = (speedPercent < 0 ? maxSpeed : speedPercent);
    // -X => Backward
    drive.controlVelocityNormalized(-spd, 0, 0);
}

void Robot::moveLeft(int speedPercent)
{
    int spd = (speedPercent < 0 ? maxSpeed : speedPercent);
    // +Y => Left
    drive.controlVelocityNormalized(0, spd, 0);
}

void Robot::moveRight(int speedPercent)
{
    int spd = (speedPercent < 0 ? maxSpeed : speedPercent);
    // -Y => Right
    drive.controlVelocityNormalized(0, -spd, 0);
}

void Robot::rotateLeft(int speedPercent)
{
    int spd = (speedPercent < 0 ? maxSpeed : speedPercent);
    // +Rotation => CCW (turn left)
    drive.controlVelocityNormalized(0, 0, spd);
}

void Robot::rotateRight(int speedPercent)
{
    int spd = (speedPercent < 0 ? maxSpeed : speedPercent);
    // -Rotation => CW (turn right)
    drive.controlVelocityNormalized(0, 0, -spd);
}

void Robot::stop()
{
    drive.controlVelocityNormalized(0, 0, 0);
}

MecanumDrive &Robot::getMecanumDrive()
{
    return drive;
}

void Robot::controlVelocityNormalized(int xPercent, int yPercent, int rotationPercent)
{
    drive.controlVelocityNormalized(xPercent, yPercent, rotationPercent);
}

void Robot::controlVelocity(float xVel_mm_s, float yVel_mm_s, float rot_rad_s)
{
    drive.controlVelocity(xVel_mm_s, yVel_mm_s, rot_rad_s);
}

void Robot::turnOnWhiteLed()
{
    ledTelltale.turnOnWhite();
}

void Robot::turnOffWhiteLed()
{
    ledTelltale.turnOffWhite();
}

void Robot::turnOnRedLed()
{
    ledTelltale.turnOnRed();
}

void Robot::turnOffRedLed()
{
    ledTelltale.turnOffRed();
}

void Robot::turnOnAllLeds()
{
    ledTelltale.turnOnAll();
}

void Robot::turnOffAllLeds()
{
    ledTelltale.turnOffAll();
}

void Robot::blinkWhiteLed(uint32_t intervalMs, float dutyCyclePercent, uint32_t times)
{
    ledTelltale.blinkWhite(intervalMs, dutyCyclePercent, times);
}

void Robot::blinkRedLed(uint32_t intervalMs, float dutyCyclePercent, uint32_t times)
{
    ledTelltale.blinkRed(intervalMs, dutyCyclePercent, times);
}
