#ifndef MECANUM_DRIVE_H
#define MECANUM_DRIVE_H

#include <string>
#include <array>
#include <algorithm>
#include "SabertoothRC.h"

class MecanumDrive
{
public:
    enum MotorIndex
    {
        FRONT_LEFT = 0,
        FRONT_RIGHT,
        REAR_LEFT,
        REAR_RIGHT,
        MOTOR_COUNT
    };

    MecanumDrive(SabertoothRC &leftController, SabertoothRC &rightController, float maxSpeed = 30.0f);

    // Low-level function: set individual motor speed.
    void setMotorSpeed(MotorIndex motor, int speedPercent);

    // Low-level drive mixing using Cartesian inputs (x, y, rotation) from (-100..100).
    void controlVelocityNormalized(int xPercent, int yPercent, int rotationPercent = 0);

    // Low-level drive mixing using Cartesian inputs (x, y, rotation) in mm/s and rad/s.
    void controlVelocity(float xVel_mm_s, float yVel_mm_s, float rot_rad_s = 0);

private:
    struct MotorChannel
    {
        SabertoothRC *controller;
        int channel; // 1 or 2
    };

    SabertoothRC &leftCtrl;
    SabertoothRC &rightCtrl;
    std::array<MotorChannel, MOTOR_COUNT> motorMap;
    float maxSpeedMagnitude;

    // Helper to clamp a value between min and max.
    inline int Clamp(int value, int minVal, int maxVal)
    {
        return std::max(std::min(value, maxVal), minVal);
    }

    // Float overload for Clamp
    inline float Clamp(float value, float minVal, float maxVal)
    {
        return std::max(std::min(value, maxVal), minVal);
    }

    int speedToPercent(float val, float maxVal, float minPct=8.f, float maxPct=100.f);
};

#endif // MECANUM_DRIVE_H
