#ifndef RC_INPUT_H
#define RC_INPUT_H

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

struct RCDebugStatus
{
    bool linkUp;
    bool valid;
    uint32_t rawBytesReceived;
    uint32_t packetsReceived;
    uint32_t lastPacketAgeMs;
};

class RCInput
{
public:
    struct Config
    {
        uint16_t channelMin = 172;
        uint16_t channelMid = 992;
        uint16_t channelMax = 1811;
        uint8_t throttleChannel = 2;
        uint8_t steerChannel = 4;
        uint8_t armChannel = 5;
        uint16_t armThreshold = 1500;
        uint32_t failsafeTimeoutMs = 100;
        float deadzone = 0.05f;
    };

    void begin(HardwareSerial& serial);
    void begin(HardwareSerial& serial, const Config& config);
    void update();

    RCCommand getCommand() const;
    bool isValid() const;
    bool isInitialized() const;
    RCDebugStatus getDebugStatus() const;

    // The CRSF library uses a free callback function, so decoded channel data
    // is forwarded back into the class through this method.
    void handleRcChannels(serialReceiverLayer::rcChannels_t* rcChannels);
    void handleRawByte(int8_t byteReceived);
    void handleLinkUp();
    void handleLinkDown();

private:
    void setSafeCommand_();

private:
    Config cfg_{};
    HardwareSerial* serial_ = nullptr;
    CRSFforArduino* crsf_ = nullptr;

    // The default values match a neutral CRSF input and keep the drive disabled
    // until a valid packet has been received.
    RCCommand cmd_ = {0.0f, 0.0f, false, false, 992, 992, 992, 992, 172, 172, 172, 172};
    elapsedMillis timeout_;
    bool initialized_ = false;
    bool linkUp_ = false;
    uint32_t rawBytesReceived_ = 0;
    uint32_t packetsReceived_ = 0;

};

#endif
