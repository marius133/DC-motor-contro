#ifndef BME680_SENSOR_H
#define BME680_SENSOR_H

#include <Arduino.h>
#include <Adafruit_BME680.h>
#include <Wire.h>

#include "BME680Reading.h"

class BME680Sensor
{
public:
    enum class Bus : uint8_t
    {
        Wire0 = 0,
        Wire1 = 1,
        Wire2 = 2,
        Wire3 = 3,
        Auto = 255
    };

    enum class Status : uint8_t
    {
        NotInitialized = 0,
        Ready,
        ReadFailed
    };

    BME680Sensor();
    ~BME680Sensor();

    struct Config
    {
        Bus bus = Bus::Auto;
        uint8_t address = 0x76;
        bool autoDetectAddress = true;
        float temperatureOffsetC = 0.0f;
        uint16_t heaterTempC = 320;
        uint16_t heaterTimeMs = 150;
        uint8_t temperatureOversampling = BME680_OS_8X;
        uint8_t humidityOversampling = BME680_OS_2X;
        uint8_t pressureOversampling = BME680_OS_4X;
        uint8_t filterSize = BME680_FILTER_SIZE_3;
    };

    bool begin();
    bool begin(const Config& config);
    bool read();

    bool isInitialized() const;
    bool isDataValid() const;
    Status status() const;
    Bus detectedBus() const;
    const char* detectedBusName() const;
    uint8_t detectedAddress() const;
    const BME680Reading& getReading() const;

private:
    bool tryBeginOnBusAtAddress_(TwoWire& wire, Bus bus, uint8_t address);
    void rebuildSensor_(TwoWire& wire);
    void applyConfig_();
    void resetReading_();

private:
    Config cfg_{};
    Adafruit_BME680* sensor_ = nullptr;
    BME680Reading reading_{}; 
    bool initialized_ = false;
    Bus detectedBus_ = Bus::Auto;
    uint8_t detectedAddress_ = 0;
    Status status_ = Status::NotInitialized;
};

#endif
