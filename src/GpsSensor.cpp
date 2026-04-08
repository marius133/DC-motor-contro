#include "GpsSensor.h"

#include <cmath>

void GpsSensor::begin(HardwareSerial& serial, uint32_t baudRate)
{
    serial_ = &serial;
    serial_->begin(baudRate);
    initialized_ = true;
    resetReading_();
}

void GpsSensor::update()
{
    if (serial_ == nullptr)
    {
        initialized_ = false;
        return;
    }

    // TinyGPS++ consumes the byte stream incrementally, so each received
    // character is fed directly into the parser.
    while (serial_->available() > 0)
    {
        gps_.encode(static_cast<char>(serial_->read()));
    }

    updateReading_();
}

bool GpsSensor::isInitialized() const
{
    return initialized_;
}

bool GpsSensor::isDataValid() const
{
    return reading_.valid;
}

const GPSReading& GpsSensor::getReading() const
{
    return reading_;
}

void GpsSensor::resetReading_()
{
    reading_.latitudeDeg = NAN;
    reading_.longitudeDeg = NAN;
    reading_.altitudeM = NAN;
    reading_.speedKnots = NAN;
    reading_.hdop = NAN;
    reading_.satellites = 0;
    reading_.year = 0;
    reading_.month = 0;
    reading_.day = 0;
    reading_.hour = 0;
    reading_.minute = 0;
    reading_.second = 0;
    reading_.timestampMs = 0;
    reading_.fix = false;
    reading_.valid = false;
}

void GpsSensor::updateReading_()
{
    // Keep invalid values as NaN so the CSV logger can leave those cells empty
    // instead of writing misleading placeholder numbers.
    if (gps_.location.isValid())
    {
        reading_.latitudeDeg = gps_.location.lat();
        reading_.longitudeDeg = gps_.location.lng();
    }
    else
    {
        reading_.latitudeDeg = NAN;
        reading_.longitudeDeg = NAN;
    }

    reading_.altitudeM = gps_.altitude.isValid() ? static_cast<float>(gps_.altitude.meters()) : NAN;
    reading_.speedKnots = gps_.speed.isValid() ? static_cast<float>(gps_.speed.knots()) : NAN;
    reading_.hdop = gps_.hdop.isValid() ? static_cast<float>(gps_.hdop.hdop()) : NAN;
    reading_.satellites = gps_.satellites.isValid() ? static_cast<uint8_t>(gps_.satellites.value()) : 0;

    if (gps_.date.isValid())
    {
        reading_.year = static_cast<uint16_t>(gps_.date.year());
        reading_.month = static_cast<uint8_t>(gps_.date.month());
        reading_.day = static_cast<uint8_t>(gps_.date.day());
    }
    else
    {
        reading_.year = 0;
        reading_.month = 0;
        reading_.day = 0;
    }

    if (gps_.time.isValid())
    {
        reading_.hour = static_cast<uint8_t>(gps_.time.hour());
        reading_.minute = static_cast<uint8_t>(gps_.time.minute());
        reading_.second = static_cast<uint8_t>(gps_.time.second());
    }
    else
    {
        reading_.hour = 0;
        reading_.minute = 0;
        reading_.second = 0;
    }

    reading_.fix = gps_.location.isValid();
    reading_.valid = gps_.location.isValid();

    if (gps_.location.isUpdated() || gps_.altitude.isUpdated() || gps_.time.isUpdated())
    {
        reading_.timestampMs = millis();
    }
}
