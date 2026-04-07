#pragma once

#include <Arduino.h>
#include <CRSFforArduino.hpp>

struct RCCommand
{
  
   float throttle;
    float steer;
    bool armed;
    bool valid;

    uint16_t rawCh1;
    uint16_t rawCh2;
    uint16_t rawCh3;
    uint16_t rawCh4;
    uint16_t rawCh5;
    uint16_t rawCh6;
    uint16_t rawCh7;
    uint16_t rawCh8;
};


class RCInput
{
public:
    void begin(HardwareSerial& serial);
    void update();

    RCCommand getCommand() const;
    bool isValid() const;

    // Må være public fordi den globale CRSF-callbacken
    // ikke er medlem av klassen og derfor ikke kan kalle private metoder.
    void handleRcChannels(serialReceiverLayer::rcChannels_t* rcChannels);

private:
    float normalizeChannel(uint16_t ch);
    float applyDeadband(float x, float deadband = 0.05f);

private:
    HardwareSerial* serial_ = nullptr;
    CRSFforArduino* crsf_   = nullptr;

    // Safe defaults before first valid RC packet arrives
   RCCommand cmd_ = {0.0f, 0.0f, false, false, 992, 992, 992, 992, 172, 172, 172, 172};
    elapsedMillis timeout_;

    static constexpr uint16_t CH_MIN = 172;
    static constexpr uint16_t CH_MID = 992;
    static constexpr uint16_t CH_MAX = 1811;

    static constexpr uint16_t ARM_THRESHOLD       = 1500;
    static constexpr uint32_t FAILSAFE_TIMEOUT_MS = 100;
    static constexpr float    DEADZONE            = 0.05f;
};