#include "rc_input.h"
#include <CRSFforArduino.hpp>

// -----------------------------------------------------------------------------
// Intern singleton-peker for callback
// CRSF callbacken er en vanlig C-funksjon, så vi trenger en bro tilbake til objektet.
// -----------------------------------------------------------------------------
static RCInput *g_rcInstance = nullptr;

// Forward declaration av callback
static void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels);

void RCInput::begin(HardwareSerial &serial)
{
    serial_ = &serial;
    g_rcInstance = this;

    // Start i safe state
    cmd_.throttle = 0.0f;
    cmd_.steer = 0.0f;
    cmd_.armed = false;
    cmd_.valid = false;
    cmd_.rawCh1 = 992;
    cmd_.rawCh2 = 992;
    cmd_.rawCh4 = 992;
    cmd_.rawCh5 = 172;

    timeout_ = 0;

    // Biblioteket støtter constructor med HardwareSerial*
    crsf_ = new CRSFforArduino(serial_);

    // begin() finnes i biblioteket, default baud = CRSF baudrate
    if (!crsf_->begin())
    {
        // Hvis init feiler, behold safe state
        delete crsf_;
        crsf_ = nullptr;
        return;
    }

    // Callback når nye RC-kanaler kommer inn
    crsf_->setRcChannelsCallback(onReceiveRcChannels);
}

void RCInput::update()
{
    if (crsf_ != nullptr)
    {
        crsf_->update();
    }

    // Timeout / failsafe hvis vi ikke har fått gyldige pakker på en stund
    if (timeout_ > FAILSAFE_TIMEOUT_MS)
    {
        cmd_.valid = false;
        cmd_.armed = false;
        cmd_.throttle = 0.0f;
        cmd_.steer = 0.0f;
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

void RCInput::handleRcChannels(serialReceiverLayer::rcChannels_t *rcChannels)
{
    if (crsf_ == nullptr || rcChannels == nullptr)
    {
        cmd_.valid = false;
        cmd_.armed = false;
        cmd_.throttle = 0.0f;
        cmd_.steer = 0.0f;
        return;
    }

    // Bibliotekets eksempel bruker rcChannels->failsafe
    if (rcChannels->failsafe)
    {
        cmd_.valid = false;
        cmd_.armed = false;
        cmd_.throttle = 0.0f;
        cmd_.steer = 0.0f;
        return;
    }

    // Les ut rå kanaler
    // Kanalnummer i bibliotek-eksempelet er 1-basert
    cmd_.rawCh1 = crsf_->getChannel(1);
    cmd_.rawCh2 = crsf_->getChannel(2);
    cmd_.rawCh3 = crsf_->getChannel(3);
    cmd_.rawCh4 = crsf_->getChannel(4);
    cmd_.rawCh5 = crsf_->getChannel(5);
    cmd_.rawCh6 = crsf_->getChannel(6);
    cmd_.rawCh7 = crsf_->getChannel(7);
    cmd_.rawCh8 = crsf_->getChannel(8);

    float steer = normalizeChannel(cmd_.rawCh4);
    float throttle = normalizeChannel(cmd_.rawCh2);

    // Deadband rundt midten så roboten ikke kryper som en rar liten bille
    steer = applyDeadband(steer, DEADZONE);
    throttle = applyDeadband(throttle, DEADZONE);

    // Arm-switch
    cmd_.armed = (cmd_.rawCh5 > ARM_THRESHOLD);

    // Hvis ikke armed, null ut kommandoer
    if (!cmd_.armed)
    {
        throttle = 0.0f;
        steer = 0.0f;
    }

    cmd_.throttle = throttle;
    cmd_.steer = steer;
    cmd_.valid = true;

    // Vi fikk en gyldig pakke nå
    timeout_ = 0;
}

float RCInput::normalizeChannel(uint16_t ch)
{
    // Typiske CRSF-kanaler:
    // min ~172, mid ~992, max ~1811
    if (ch >= CH_MID)
    {
        return static_cast<float>(ch - CH_MID) / static_cast<float>(CH_MAX - CH_MID);
    }
    else
    {
        return static_cast<float>(ch - CH_MID) / static_cast<float>(CH_MID - CH_MIN);
    }
}

float RCInput::applyDeadband(float x, float deadband)
{
    if (fabsf(x) < deadband)
    {
        return 0.0f;
    }

    return x;
}

// -----------------------------------------------------------------------------
// Statisk callback-funksjon som biblioteket kaller
// -----------------------------------------------------------------------------
static void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels)
{
    if (g_rcInstance != nullptr)
    {
        g_rcInstance->handleRcChannels(rcChannels);
    }
}