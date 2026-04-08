#ifndef GPS_SENSOR_H
#define GPS_SENSOR_H

#include <Arduino.h>
#include <TinyGPS++.h>

struct GPSReading
{
    double latitudeDeg;
    double longitudeDeg;
    float altitudeM;
    float speedKnots;
    float hdop;
    uint8_t satellites;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint32_t timestampMs;
    bool fix;
    bool valid;
};

class GpsSensor
{
public:
    void begin(HardwareSerial& serial, uint32_t baudRate = 9600);
    void update();

    bool isInitialized() const;
    bool isDataValid() const;
    const GPSReading& getReading() const;

private:
    // Copies the latest decoded TinyGPS++ values into the project-specific
    // reading struct used by the logger and status output.
    void resetReading_();
    void updateReading_();

private:
    HardwareSerial* serial_ = nullptr;
    TinyGPSPlus gps_;
    GPSReading reading_{};
    bool initialized_ = false;
};

#endif
