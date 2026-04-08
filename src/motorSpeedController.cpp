#include "motorSpeedController.h"

#include <cmath>

MotorSpeedController::MotorSpeedController(const MotorDriver::DriverPins& motorPins,
                                           const MotorDriver::DriverConfig& motorCfg,
                                           uint8_t encoderPinA,
                                           uint8_t encoderPinB,
                                           const EncoderDriver::config& encoderCfg,
                                           const SpeedControlConfig& speedCfg)
    : encoder_(encoderPinA, encoderPinB),
      encoderDriver_(encoder_, encoderCfg),
      motor_(motorPins, motorCfg),
      cfg_(speedCfg)
{
}

void MotorSpeedController::begin()
{
    motor_.begin();
    motor_.enable(false);
    encoderDriver_.zero();
    resetController_();
}

void MotorSpeedController::enable(bool on)
{
    if (!on)
    {
        targetSpeedCountsPerSec_ = 0.0f;
        motor_.enable(false);
        enabled_ = false;
        resetController_();
        return;
    }

    motor_.enable(true);
    enabled_ = true;
}

bool MotorSpeedController::isEnabled() const
{
    return enabled_;
}

void MotorSpeedController::setTargetNormalized(float normalized)
{
    normalized = constrain(normalized, -1.0f, 1.0f);
    setTargetSpeedCountsPerSec(normalized * cfg_.maxTargetSpeedCountsPerSec);
}

void MotorSpeedController::setTargetSpeedCountsPerSec(float targetSpeedCountsPerSec)
{
    const float limit = fabsf(cfg_.maxTargetSpeedCountsPerSec);
    targetSpeedCountsPerSec_ = constrain(targetSpeedCountsPerSec, -limit, limit);
}

void MotorSpeedController::update(uint32_t dtUs)
{
    encoderDriver_.update(dtUs);
    measuredSpeedCountsPerSec_ = encoderDriver_.speed_counts_per_sec();

    if (!enabled_ || dtUs == 0)
    {
        return;
    }

    const float dtS = static_cast<float>(dtUs) * 1.0e-6f;
    const float error = targetSpeedCountsPerSec_ - measuredSpeedCountsPerSec_;

    integral_ += error * dtS;
    if (cfg_.ki > 0.0f && cfg_.integralLimit > 0.0f)
    {
        const float maxIntegralState = cfg_.integralLimit / cfg_.ki;
        integral_ = constrain(integral_, -maxIntegralState, maxIntegralState);
    }

    float derivative = 0.0f;
    if (hasPreviousError_)
    {
        derivative = (error - previousError_) / dtS;
    }

    float command = (cfg_.kf * targetSpeedCountsPerSec_) +
                    (cfg_.kp * error) +
                    (cfg_.ki * integral_) +
                    (cfg_.kd * derivative);

    command = constrain(command, -1.0f, 1.0f);
    motor_.setCommand(command);

    previousError_ = error;
    lastCommand_ = command;
    hasPreviousError_ = true;
}

void MotorSpeedController::stop()
{
    targetSpeedCountsPerSec_ = 0.0f;
    motor_.stop();
    resetController_();
}

float MotorSpeedController::targetSpeedCountsPerSec() const
{
    return targetSpeedCountsPerSec_;
}

float MotorSpeedController::measuredSpeedCountsPerSec() const
{
    return measuredSpeedCountsPerSec_;
}

float MotorSpeedController::lastCommand() const
{
    return lastCommand_;
}

int32_t MotorSpeedController::encoderCount() const
{
    return encoderDriver_.count();
}

void MotorSpeedController::resetController_()
{
    integral_ = 0.0f;
    previousError_ = 0.0f;
    lastCommand_ = 0.0f;
    hasPreviousError_ = false;
}
