#ifndef MOTOR_SPEED_CONTROLLER_H
#define MOTOR_SPEED_CONTROLLER_H

#include <Arduino.h>
#include <Encoder.h>

#include "encoderDriver.h"
#include "motorDriver.h"

class MotorSpeedController
{
public:
    struct SpeedControlConfig
    {
        float maxTargetSpeedCountsPerSec = 4000.0f;
        float kp = 0.0002f;
        float ki = 0.0f;
        float kd = 0.0f;
        float kf = 0.0f;
        float integralLimit = 1.0f;
    };

    MotorSpeedController(const MotorDriver::DriverPins& motorPins,
                         const MotorDriver::DriverConfig& motorCfg,
                         uint8_t encoderPinA,
                         uint8_t encoderPinB,
                         const EncoderDriver::config& encoderCfg,
                         const SpeedControlConfig& speedCfg);

    void begin();
    void enable(bool on);
    bool isEnabled() const;

    void setTargetNormalized(float normalized);
    void setTargetSpeedCountsPerSec(float targetSpeedCountsPerSec);
    void update(uint32_t dtUs);
    void stop();

    float targetSpeedCountsPerSec() const;
    float measuredSpeedCountsPerSec() const;
    float lastCommand() const;
    int32_t encoderCount() const;

private:
    void resetController_();

private:
    Encoder encoder_;
    EncoderDriver encoderDriver_;
    MotorDriver motor_;
    SpeedControlConfig cfg_;
    bool enabled_ = false;
    float targetSpeedCountsPerSec_ = 0.0f;
    float measuredSpeedCountsPerSec_ = 0.0f;
    float integral_ = 0.0f;
    float previousError_ = 0.0f;
    float lastCommand_ = 0.0f;
    bool hasPreviousError_ = false;
};

#endif
