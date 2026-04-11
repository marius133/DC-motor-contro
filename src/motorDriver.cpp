#include "motorDriver.h"

#include <cmath>
#include <limits>

MotorDriver::MotorDriver(const DriverPins& pins, const DriverConfig& cfg)
    : pins_(pins), cfg_(cfg)
{
}

void MotorDriver::begin()
{
    // The driver starts disabled and with zero PWM so the motors remain idle
    // during system startup.
    pinMode(pins_.EN1, OUTPUT);
    pinMode(pins_.EN2, OUTPUT);
    pinMode(pins_.PWM1, OUTPUT);
    pinMode(pins_.PWM2, OUTPUT);

    analogWriteResolution(cfg_.pwm_resolution_bits);
    analogWriteFrequency(pins_.PWM1, cfg_.pwm_freq_hz);
    analogWriteFrequency(pins_.PWM2, cfg_.pwm_freq_hz);

    enabled_ = false;
    lastCommand_ = 0.0f;
    lastPwm_ = 0;

    safeStop_();
    digitalWrite(pins_.EN1, LOW);
    digitalWrite(pins_.EN2, LOW);
}

void MotorDriver::enable(bool on)
{
    if (on)
    {
        digitalWrite(pins_.EN1, HIGH);
        digitalWrite(pins_.EN2, HIGH);
        enabled_ = true;
        return;
    }

    safeStop_();
    digitalWrite(pins_.EN1, LOW);
    digitalWrite(pins_.EN2, LOW);
    enabled_ = false;
    lastCommand_ = 0.0f;
    lastPwm_ = 0;
}

bool MotorDriver::isEnabled() const
{
    return enabled_;
}

void MotorDriver::setCommand(float u)
{
    if (!enabled_)
    {
        safeStop_();
        lastCommand_ = 0.0f;
        lastPwm_ = 0;
        return;
    }

    if (u > 1.0f)
    {
        u = 1.0f;
    }
    else if (u < -1.0f)
    {
        u = -1.0f;
    }

    if (cfg_.invert)
    {
        u = -u;
    }

    if (fabsf(u) < cfg_.deadzone)
    {
        u = 0.0f;
    }

    const bool directionChanged = ((lastCommand_ > 0.0f) && (u < 0.0f)) ||
                                  ((lastCommand_ < 0.0f) && (u > 0.0f));
    if (directionChanged)
    {
        // Insert a short neutral interval before reversing direction to reduce
        // current spikes in the bridge and motor.
        safeStop_();
        delayMicroseconds(300);
    }

    const int duty = commandToPwm_(u);
    if (u > 0.0f)
    {
        writePwm_(duty, 0);
    }
    else if (u < 0.0f)
    {
        writePwm_(0, duty);
    }
    else
    {
        safeStop_();
    }

    lastCommand_ = u;
    lastPwm_ = duty;
}

void MotorDriver::stop()
{
    safeStop_();
    lastCommand_ = 0.0f;
    lastPwm_ = 0;
}

void MotorDriver::brake()
{
    stop();
}

float MotorDriver::lastCommand() const
{
    return lastCommand_;
}

float MotorDriver::readCurrent_mA() const
{
    return std::numeric_limits<float>::quiet_NaN();
}

void MotorDriver::writePwm_(int forwardPwm, int reversePwm)
{
    if (forwardPwm < 0)
    {
        forwardPwm = 0;
    }

    if (reversePwm < 0)
    {
        reversePwm = 0;
    }

    analogWrite(pins_.PWM1, forwardPwm);
    analogWrite(pins_.PWM2, reversePwm);
}

int MotorDriver::commandToPwm_(float u) const
{
    const float amplitude = fabsf(u);
    const uint32_t maxDuty = (1UL << cfg_.pwm_resolution_bits) - 1UL;
    const float configuredMinimumDrive = (u > 0.0f) ? cfg_.minimumForwardDriveCommand : cfg_.minimumReverseDriveCommand;
    const float minimumDriveCommand = constrain(configuredMinimumDrive, 0.0f, 1.0f);

    float effectiveAmplitude = amplitude;
    if (amplitude > 0.0f && minimumDriveCommand > amplitude)
    {
        effectiveAmplitude = minimumDriveCommand;
    }

    int duty = static_cast<int>(lroundf(effectiveAmplitude * static_cast<float>(maxDuty)));
    if (duty < 0)
    {
        duty = 0;
    }

    if (static_cast<uint32_t>(duty) > maxDuty)
    {
        duty = static_cast<int>(maxDuty);
    }

    return duty;
}

void MotorDriver::safeStop_()
{
    writePwm_(0, 0);
}
