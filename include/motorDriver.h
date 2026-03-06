#pragma once
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
        uint8_t IS2 = -1;
        uint8_t IS1 = 1;
    };

   struct DriverConfig
{
    uint32_t pwm_freq_hz;
    uint8_t  pwm_resolution_bits;
    bool     invert;
    float    deadzone;

    DriverConfig()
        : pwm_freq_hz(20000),
          pwm_resolution_bits(12),
          invert(false),
          deadzone(0.0f)
    {}
};

    MotorDriver(const DriverPins &pins, const DriverConfig &cfg = DriverConfig{});
    void begin();
    void enable(bool on);
    bool isEnabled() const;
    void setCommand(float u);
    void stop();
    float lastCommand() const;
    void brake();
    float readCurrent_mA() const;

private:
    DriverPins _pins;
    DriverConfig _cfg;

    bool _enabled = false;
    float _u_last = 0.0f;
    int _pwm_last = 0;
    void writePwm_(int rpwm, int lpwm);
    int commandToPwm_(float u) const;
    void safeStop_();
};
