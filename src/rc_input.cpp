#include "rc_input.h"

#include <cmath>

namespace
{
RCInput* g_rcInstance = nullptr;

float normalizeChannel(uint16_t ch, const RCInput::Config& cfg)
{
    if (ch >= cfg.channelMid)
    {
        return static_cast<float>(ch - cfg.channelMid) / static_cast<float>(cfg.channelMax - cfg.channelMid);
    }

    return static_cast<float>(ch - cfg.channelMid) / static_cast<float>(cfg.channelMid - cfg.channelMin);
}

float applyDeadband(float x, float deadband)
{
    if (fabsf(x) < deadband)
    {
        return 0.0f;
    }

    return x;
}

// The CRSF callback is a free function, so this bridge forwards new packets
// to the active RCInput instance.
void onReceiveRcChannels(serialReceiverLayer::rcChannels_t* rcChannels)
{
    if (g_rcInstance != nullptr)
    {
        g_rcInstance->handleRcChannels(rcChannels);
    }
}

void onReceiveRawData(int8_t byteReceived)
{
    if (g_rcInstance != nullptr)
    {
        g_rcInstance->handleRawByte(byteReceived);
    }
}

void onLinkUp()
{
    if (g_rcInstance != nullptr)
    {
        g_rcInstance->handleLinkUp();
    }
}

void onLinkDown()
{
    if (g_rcInstance != nullptr)
    {
        g_rcInstance->handleLinkDown();
    }
}
} // namespace

void RCInput::begin(HardwareSerial& serial)
{
    begin(serial, Config{});
}

void RCInput::begin(HardwareSerial& serial, const Config& config)
{
    serial_ = &serial;
    cfg_ = config;
    g_rcInstance = this;
    setSafeCommand_();
    timeout_ = 0;
    initialized_ = false;
    linkUp_ = false;
    rawBytesReceived_ = 0;
    packetsReceived_ = 0;

    crsf_ = new CRSFforArduino(serial_);
    if (!crsf_->begin())
    {
        delete crsf_;
        crsf_ = nullptr;
        return;
    }

    crsf_->setRcChannelsCallback(onReceiveRcChannels);
    crsf_->setRawDataCallback(onReceiveRawData);
    crsf_->setLinkUpCallback(onLinkUp);
    crsf_->setLinkDownCallback(onLinkDown);
    initialized_ = true;
}

void RCInput::update()
{
    if (crsf_ != nullptr)
    {
        crsf_->update();
    }

    // Drop back to a neutral command if packets stop arriving within the
    // expected failsafe window.
    if (timeout_ > cfg_.failsafeTimeoutMs)
    {
        setSafeCommand_();
    }
}

RCCommand RCInput::getCommand() const
{
    return cmd_;
}

bool RCInput::isValid() const
{
    return cmd_.valid;
}

bool RCInput::isInitialized() const
{
    return initialized_;
}

RCDebugStatus RCInput::getDebugStatus() const
{
    return {linkUp_, cmd_.valid, rawBytesReceived_, packetsReceived_, static_cast<uint32_t>(timeout_)};
}

void RCInput::handleRcChannels(serialReceiverLayer::rcChannels_t* rcChannels)
{
    if (crsf_ == nullptr || rcChannels == nullptr || rcChannels->failsafe)
    {
        setSafeCommand_();
        return;
    }

    // The library exposes channels using 1-based indexing.
    cmd_.rawCh1 = crsf_->getChannel(1);
    cmd_.rawCh2 = crsf_->getChannel(2);
    cmd_.rawCh3 = crsf_->getChannel(3);
    cmd_.rawCh4 = crsf_->getChannel(4);
    cmd_.rawCh5 = crsf_->getChannel(5);
    cmd_.rawCh6 = crsf_->getChannel(6);
    cmd_.rawCh7 = crsf_->getChannel(7);
    cmd_.rawCh8 = crsf_->getChannel(8);

    const auto readChannel = [this](uint8_t channel) -> uint16_t
    {
        if (channel < 1 || channel > 8)
        {
            return cfg_.channelMid;
        }

        return crsf_->getChannel(channel);
    };

    float throttle = applyDeadband(normalizeChannel(readChannel(cfg_.throttleChannel), cfg_), cfg_.deadzone);
    float steer = applyDeadband(normalizeChannel(readChannel(cfg_.steerChannel), cfg_), cfg_.deadzone);

    // The arm channel gates all motion commands so that the motors remain
    // disabled until the operator explicitly enables them.
    cmd_.armed = (readChannel(cfg_.armChannel) > cfg_.armThreshold);
    if (!cmd_.armed)
    {
        throttle = 0.0f;
        steer = 0.0f;
    }

    cmd_.throttle = throttle;
    cmd_.steer = steer;
    cmd_.valid = true;
    linkUp_ = true;
    ++packetsReceived_;
    timeout_ = 0;
}

void RCInput::handleRawByte(int8_t byteReceived)
{
    (void)byteReceived;
    ++rawBytesReceived_;
}

void RCInput::handleLinkUp()
{
    linkUp_ = true;
}

void RCInput::handleLinkDown()
{
    linkUp_ = false;
}

void RCInput::setSafeCommand_()
{
    // Safe state is used both at startup and after failsafe events.
    cmd_.throttle = 0.0f;
    cmd_.steer = 0.0f;
    cmd_.armed = false;
    cmd_.valid = false;
    cmd_.rawCh1 = cfg_.channelMid;
    cmd_.rawCh2 = cfg_.channelMid;
    cmd_.rawCh3 = cfg_.channelMid;
    cmd_.rawCh4 = cfg_.channelMid;
    cmd_.rawCh5 = cfg_.channelMin;
    cmd_.rawCh6 = cfg_.channelMin;
    cmd_.rawCh7 = cfg_.channelMin;
    cmd_.rawCh8 = cfg_.channelMin;
}
