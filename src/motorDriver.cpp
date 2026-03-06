// MotorDriver.cpp
#include "MotorDriver.h"
#include <Arduino.h>
#include <cmath>   // fabsf, lroundf
#include <limits>  // quiet_NaN

MotorDriver::MotorDriver(const DriverPins &pins, const DriverConfig &cfg)
    : _pins(pins), _cfg(cfg)
{
}

void MotorDriver::begin()
{
    // PinMode-setup
    pinMode(_pins.EN1, OUTPUT);
    pinMode(_pins.EN2, OUTPUT);
    pinMode(_pins.PWM1, OUTPUT);
    pinMode(_pins.PWM2, OUTPUT);

    // Optional: IS pins (hvis du vil bruke de senere)
    // pinMode(_pins.IS1, INPUT);
    // pinMode(_pins.IS2, INPUT);

    // PWM-setup (Teensy)
    analogWriteResolution(_cfg.pwm_resolution_bits);
    analogWriteFrequency(_pins.PWM1, _cfg.pwm_freq_hz);
    analogWriteFrequency(_pins.PWM2, _cfg.pwm_freq_hz);

    // Start i safe state
    _enabled = false;
    _u_last = 0.0f;
    _pwm_last = 0;

    safeStop_();  // PWM=0
    digitalWrite(_pins.EN1, LOW);
    digitalWrite(_pins.EN2, LOW);
}

void MotorDriver::enable(bool on)
{
    if (on) {
        digitalWrite(_pins.EN1, HIGH);
        digitalWrite(_pins.EN2, HIGH);
        _enabled = true;
    } else {
        safeStop_();
        digitalWrite(_pins.EN1, LOW);
        digitalWrite(_pins.EN2, LOW);
        _enabled = false;
        _u_last = 0.0f;
        _pwm_last = 0;
    }
}

bool MotorDriver::isEnabled() const
{
    return _enabled;
}

void MotorDriver::setCommand(float u)
{
    // Hvis disabled: aldri la PWM ligge og surre
    if (!_enabled) {
        safeStop_();
        _u_last = 0.0f;
        _pwm_last = 0;
        return;
    }

    // Clamp til [-1, 1]
    if (u > 1.0f)  u = 1.0f;
    if (u < -1.0f) u = -1.0f;

    // Invert (bytter fram/bak)
    if (_cfg.invert) u = -u;

    // Deadzone
    if (fabsf(u) < _cfg.deadzone) u = 0.0f;

    // Retningsbytte-beskyttelse: stopp før bytte
    // (hindrer brutale reverseringer)
    const bool lastPos = (_u_last > 0.0f);
    const bool lastNeg = (_u_last < 0.0f);
    const bool newPos  = (u > 0.0f);
    const bool newNeg  = (u < 0.0f);

    if ((lastPos && newNeg) || (lastNeg && newPos)) {
        safeStop_();
        delayMicroseconds(300); // liten pause (kan justeres)
    }

    // Konverter kommando -> PWM duty
    const int duty = commandToPwm_(u);

    // PWM1 = "forward", PWM2 = "reverse"
    if (u > 0.0f) {
        writePwm_(duty, 0);
    } else if (u < 0.0f) {
        writePwm_(0, duty);
    } else {
        safeStop_();
    }

    _u_last = u;
    _pwm_last = duty;
}

void MotorDriver::stop()
{
    // Coast stop: PWM=0, EN kan stå HIGH
    safeStop_();
    _u_last = 0.0f;
    _pwm_last = 0;
}

float MotorDriver::lastCommand() const
{
    return _u_last;
}

void MotorDriver::brake()
{
    // Første testversjon: samme som stop()
    stop();
}

// float MotorDriver::readCurrent_mA()
// {
//     // IS1/IS2 på disse modulene er ofte uklare/ikke kalibrert.
//     // Returner NaN til du har bestemt skala og faktisk bruker IS.
//     return std::numeric_limits<float>::quiet_NaN();
// }

// ---- Private helpers ----
void MotorDriver::writePwm_(int rpwm, int lpwm)
{
    // Sikkerhet: ingen negative verdier
    if (rpwm < 0) rpwm = 0;
    if (lpwm < 0) lpwm = 0;

    // Skriv PWM
    analogWrite(_pins.PWM1, rpwm);
    analogWrite(_pins.PWM2, lpwm);
}

int MotorDriver::commandToPwm_(float u) const
{
    // abs(u) i [0..1] -> duty i [0..maxDuty]
    const float au = fabsf(u);

    // maxDuty = (1<<bits)-1
    // (bits er uint8_t, så dette er trygt)
    const uint32_t maxDuty = (1UL << _cfg.pwm_resolution_bits) - 1UL;

    int duty = (int)lroundf(au * (float)maxDuty);
    if (duty < 0) duty = 0;
    if ((uint32_t)duty > maxDuty) duty = (int)maxDuty;

    return duty;
}

void MotorDriver::safeStop_()
{
    writePwm_(0, 0);
}