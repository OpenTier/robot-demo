#include "MecanumDrive.h"
#include <cmath>
#include <algorithm>

/**
 * @brief Map wheel velocity (val) in mm/s to a percent [-100..+100], 
 *        ensuring that any nonzero command is at least +/-10%.
 *
 * @param val    Wheel speed in mm/s (could be negative or positive).
 * @param maxVal Maximum wheel speed (mm/s) that should map to 100% magnitude.
 * @param minPct The minimum absolute % to use for any nonzero speed (e.g. 10).
 * @param maxPct The maximum absolute % for the top speed (e.g. 100).
 * @return int   The mapped percent in [-maxPct..maxPct].
 *
 * If val=0, outputs 0. 
 * If abs(val) > maxVal, saturates to +/-maxPct.
 * Otherwise linearly maps abs(val) from [0..maxVal] to [minPct..maxPct].
 */
int MecanumDrive::speedToPercent(float val, float maxVal, float minPct, float maxPct)
{
    // If speed is effectively 0, return 0% directly.
    if (fabs(val) < 1e-6) {
        return 0;
    }

    float sign = (val >= 0.0f) ? 1.0f : -1.0f;
    float absVal = fabs(val);

    // Clip speed to [0..maxVal] so we don't exceed our linear range
    if (absVal > maxVal) {
        absVal = maxVal;
    }

    // ratio in [0..1]
    float ratio = absVal / maxVal;

    // Map ratio=0 => minPct, ratio=1 => maxPct:
    // outPct in [minPct..maxPct]
    float outPct = minPct + (maxPct - minPct) * ratio;

    // Return with correct sign
    return static_cast<int>(sign * outPct);
}

MecanumDrive::MecanumDrive(SabertoothRC &leftController, SabertoothRC &rightController, float maxSpeed)
    : leftCtrl(leftController), rightCtrl(rightController)
{
    // Map each motor index to the corresponding controller and channel.
    motorMap[FRONT_LEFT] = {&leftCtrl, 1};
    motorMap[REAR_LEFT] = {&leftCtrl, 2};
    motorMap[FRONT_RIGHT] = {&rightCtrl, 1};
    motorMap[REAR_RIGHT] = {&rightCtrl, 2};

    // Initialize all motors to 0 speed.
    setMotorSpeed(FRONT_LEFT, 0);
    setMotorSpeed(FRONT_RIGHT, 0);
    setMotorSpeed(REAR_LEFT, 0);
    setMotorSpeed(REAR_RIGHT, 0);

    // Set the maximum speed magnitud between -100 and 100
    maxSpeedMagnitude = Clamp(maxSpeed, -100, 100);

}

void MecanumDrive::setMotorSpeed(MotorIndex motor, int speedPercent)
{
    speedPercent = Clamp(speedPercent, -maxSpeedMagnitude, maxSpeedMagnitude);
    if (motor >= 0 && motor < MOTOR_COUNT)
    {
        SabertoothRC *ctrl = motorMap[motor].controller;
        int channel = motorMap[motor].channel;
        if (ctrl)
        {
            ctrl->setMotorSpeed(channel, speedPercent);
        }
    }
}

/**
 * @brief Drive the Mecanum robot using percentage-based commands,
 *        with the convention:
 *        +xPercent = forward,
 *        +yPercent = left,
 *        +rotationPercent = CCW rotation.
 *
 * @param xPercent      Range: -maxSpeedMagnitude..+maxSpeedMagnitude (forward/backward)
 * @param yPercent      Range: -maxSpeedMagnitude..+maxSpeedMagnitude (left/right)
 * @param rotationPercent Range: -maxSpeedMagnitude..+maxSpeedMagnitude (CCW/CW turn)
 */
void MecanumDrive::controlVelocityNormalized(int xPercent, int yPercent, int rotationPercent)
{
    // Clamp inputs to avoid exceeding your max.
    xPercent       = Clamp(xPercent, -maxSpeedMagnitude, maxSpeedMagnitude);
    yPercent       = Clamp(yPercent, -maxSpeedMagnitude, maxSpeedMagnitude);
    rotationPercent = Clamp(rotationPercent, -maxSpeedMagnitude, maxSpeedMagnitude);

    // -------------------------------------------------------------
    // Standard Mecanum formula (assuming +X=forward, +Y=left, +rot=CCW)
    // front-left  wheel =  x + y + rot
    // front-right wheel =  x - y - rot
    // rear-left   wheel =  x - y + rot
    // rear-right  wheel =  x + y - rot
    // -------------------------------------------------------------
    float fl = (float)xPercent + (float)yPercent + (float)rotationPercent;
    float fr = (float)xPercent - (float)yPercent - (float)rotationPercent;
    float rl = (float)xPercent - (float)yPercent + (float)rotationPercent;
    float rr = (float)xPercent + (float)yPercent - (float)rotationPercent;

    // Normalize if any magnitude exceeds the max speed allowed.
    float maxMag = std::max({fabs(fl), fabs(fr), fabs(rl), fabs(rr)});
    if (maxMag > maxSpeedMagnitude)
    {
        float scale = (float)maxSpeedMagnitude / maxMag;
        fl *= scale;
        fr *= scale;
        rl *= scale;
        rr *= scale;
    }

    // Convert to integer motor commands
    int fl_speed = static_cast<int>(std::round(fl));
    int fr_speed = static_cast<int>(std::round(fr));
    int rl_speed = static_cast<int>(std::round(rl));
    int rr_speed = static_cast<int>(std::round(rr));

    // Send commands to motors
    setMotorSpeed(FRONT_LEFT,  fl_speed);
    setMotorSpeed(FRONT_RIGHT, fr_speed);
    setMotorSpeed(REAR_LEFT,   rl_speed);
    setMotorSpeed(REAR_RIGHT,  rr_speed);
}

void MecanumDrive::controlVelocity(float xVel_mm_s, float yVel_mm_s, float rot_rad_s)
{
    static constexpr float L = 150.0f;
    static constexpr float W = 150.0f;
    float R = (L + W) * 0.5f;

    // "Standard" Mecanum formula for +X fwd, +Y left, +rot CCW
    float fl = xVel_mm_s - yVel_mm_s - rot_rad_s * R;
    float fr = xVel_mm_s + yVel_mm_s + rot_rad_s * R;
    float rl = xVel_mm_s + yVel_mm_s - rot_rad_s * R;
    float rr = xVel_mm_s - yVel_mm_s + rot_rad_s * R;

    // We'll assume your top speed is 1000 mm/s for each wheel
    // (adjust as needed)
    static constexpr float MAX_WHEEL_SPEED_MM_S = 1000.0f;

    // If any wheel is above max, we scale all
    float maxVal = std::max({fabs(fl), fabs(fr), fabs(rl), fabs(rr)});
    if (maxVal > MAX_WHEEL_SPEED_MM_S)
    {
        float scale = MAX_WHEEL_SPEED_MM_S / maxVal;
        fl *= scale;
        fr *= scale;
        rl *= scale;
        rr *= scale;
    }

    // Now convert each wheel speed [mm/s] -> percent, 
    // with a min. nonzero percent = 10% (for example).
    int fl_speed = speedToPercent(fl, MAX_WHEEL_SPEED_MM_S, /*minPct=*/15.f, /*maxPct=*/100.f);
    int fr_speed = speedToPercent(fr, MAX_WHEEL_SPEED_MM_S, 8.f, 100.f);
    int rl_speed = speedToPercent(rl, MAX_WHEEL_SPEED_MM_S, 8.f, 100.f);
    int rr_speed = speedToPercent(rr, MAX_WHEEL_SPEED_MM_S, 8.f, 100.f);

    // Send to motors
    setMotorSpeed(FRONT_LEFT,  fl_speed);
    setMotorSpeed(FRONT_RIGHT, fr_speed);
    setMotorSpeed(REAR_LEFT,   rl_speed);
    setMotorSpeed(REAR_RIGHT,  rr_speed);
}
