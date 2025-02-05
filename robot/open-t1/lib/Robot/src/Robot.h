#ifndef ROBOT_H
#define ROBOT_H

#include "SabertoothRC.h"
#include "SolenoidKicker.h"
#include "BatterySensor.h"
#include "MecanumDrive.h"
#include "LedTelltale.h"
#include "esp_err.h"
#include <stdint.h>
#include <string>

/**
 * @brief The Robot class encapsulates motor control, battery sensing,
 *        solenoid kicking, movement commands, LED control, etc.
 *
 * All hardware pin values are passed via the constructor.
 */
class Robot
{
public:
    /**
     * @brief Construct a new Robot object.
     *
     * @param pwmLeftFront   Pin for left controller front motor.
     * @param pwmLeftRear    Pin for left controller rear motor.
     * @param pwmRightFront  Pin for right controller front motor.
     * @param pwmRightRear   Pin for right controller rear motor.
     * @param solenoidGatePin Pin for the solenoid kicker.
     * @param defaultKickMs  Default kick duration in ms.
     * @param batteryAdcUnit ADC unit for battery sensing.
     * @param batteryAdcChannel ADC channel for battery sensing.
     * @param batteryDivider Voltage divider ratio.
     * @param whiteLedPin    GPIO pin for the white LED.
     * @param redLedPin      GPIO pin for the red LED.
     */
    Robot(int pwmLeftFront, int pwmLeftRear,
          int pwmRightFront, int pwmRightRear,
          gpio_num_t solenoidGatePin, int defaultKickMs,
          adc_unit_t batteryAdcUnit, adc_channel_t batteryAdcChannel, float batteryDivider,
          gpio_num_t whiteLedPin, gpio_num_t redLedPin,
          int setMaxSpeed);

    /**
     * @brief Initialize all hardware components (motors, kicker, battery, LEDs).
     *
     * @return esp_err_t ESP_OK on success, error code otherwise.
     */
    esp_err_t init();

    /**
     * @brief Periodically update sensor data.
     */
    void update();

    /**
     * @brief Get the current battery voltage.
     *
     * @return float Voltage in volts.
     */
    float getBatteryVoltage() const;

    /**
     * @brief Activate the kicker for a given duration (in milliseconds).
     *
     * @param kickDuration Duration in ms. If -1, uses default.
     */
    void kick(int kickDuration = -1);

    // Higher-level movement functions (if speedPercent is -1, use default speed).
    void moveForward(int speedPercent = -1);
    void moveBackward(int speedPercent = -1);
    void moveLeft(int speedPercent = -1);
    void moveRight(int speedPercent = -1);
    void rotateLeft(int speedPercent = -1);
    void rotateRight(int speedPercent = -1);
    void stop();

    /**
     * @brief Access the underlying drive system.
     *
     * @return MecanumDrive& Reference to the drive system.
     */
    MecanumDrive &getMecanumDrive();

    /**
     * @brief Allows direct Cartesian control for x, y, and rotation percentages.
     *        (each in range -100..100).
     *
     * @param xPercent        Lateral motion (-100..100).
     * @param yPercent        Forward/back motion (-100..100).
     * @param rotationPercent Rotation (-100..100).
     */
    void controlVelocityNormalized(int xPercent, int yPercent, int rotationPercent = 0);

    /**
     * @brief Move using real-world linear velocities (mm/s) and angular velocity (rad/s).
     */
    void controlVelocity(float xVel_mm_s, float yVel_mm_s, float rot_rad_s);

    // --------------------
    // LED control methods
    // --------------------
    void turnOnWhiteLed();
    void turnOffWhiteLed();
    void turnOnRedLed();
    void turnOffRedLed();
    void turnOnAllLeds();
    void turnOffAllLeds();

    /**
     * @brief Non-blocking blink for white LED, default duty 50%
     * @param intervalMs Period of one blink cycle
     * @param dutyCyclePercent 0..100 for how much of cycle is ON
     * @param times How many cycles (0 = infinite)
     */
    void blinkWhiteLed(uint32_t intervalMs, float dutyCyclePercent = 50.0f, uint32_t times = 0);

    /**
     * @brief Non-blocking blink for red LED, default duty 50%
     */
    void blinkRedLed(uint32_t intervalMs, float dutyCyclePercent = 50.0f, uint32_t times = 0);

private:
    // Motor controllers, kicker, battery, drive...
    SabertoothRC leftController;
    SabertoothRC rightController;
    SolenoidKicker kicker;
    BatterySensor battery;
    MecanumDrive drive;
    LedTelltale ledTelltale;

    int maxSpeed;
};

#endif // ROBOT_H
