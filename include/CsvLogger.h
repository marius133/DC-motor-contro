#ifndef CSV_LOGGER_H
#define CSV_LOGGER_H

#include <Arduino.h>
#include <SD.h>

#include "BME680Reading.h"
#include "GpsSensor.h"

struct LogRecord
{
    uint32_t deviceTimestampMs;
    BME680Reading environment;
    GPSReading gps;
};

class CsvLogger
{
public:
    bool begin(uint8_t chipSelect = BUILTIN_SDCARD, const char* filename = "env_log.csv");
    bool append(const LogRecord& record);

    bool isInitialized() const;
    const char* filename() const;

private:
    // Empty CSV fields are used when a measurement is unavailable, which keeps
    // the file easy to import in GIS and analysis tools.
    bool writeHeader_(File& file);
    void printDateTime_(File& file, const GPSReading& gps);
    void printDoubleOrBlank_(File& file, double value, uint8_t precision);
    void printFloatOrBlank_(File& file, float value, uint8_t precision);
    void printUintOrBlank_(File& file, uint32_t value, bool valid);

private:
    uint8_t chipSelect_ = BUILTIN_SDCARD;
    const char* filename_ = "env_log.csv";
    bool initialized_ = false;
};

#endif
