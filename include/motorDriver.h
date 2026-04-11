#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

class MotorDriver
{
public:
    struct DriverPins
    {
        uint8_t EN2;
        uint8_t EN1;
        uint8_t PWM1;
        uint8_t PWM2;
        uint8_t IS2 = 255;
        uint8_t IS1 = 255;
    };

    struct DriverConfig
    {
        uint32_t pwm_freq_hz = 20000;
        uint8_t pwm_resolution_bits = 12;
        bool invert = false;
        float deadzone = 0.0f;
        float minimumForwardDriveCommand = 0.0f;
        float minimumReverseDriveCommand = 0.0f;
    };

    MotorDriver(const DriverPins& pins, const DriverConfig& cfg);

    void begin();
    void enable(bool on);
    bool isEnabled() const;

    void setCommand(float u);
    void stop();
    void brake();

    float lastCommand() const;
    float readCurrent_mA() const;

private:
    // The two PWM outputs represent forward and reverse drive for the H-bridge.
    void writePwm_(int forwardPwm, int reversePwm);
    int commandToPwm_(float u) const;
    void safeStop_();

private:
    DriverPins pins_;
    DriverConfig cfg_;
    bool enabled_ = false;
    float lastCommand_ = 0.0f;
    int lastPwm_ = 0;
};

#endif
