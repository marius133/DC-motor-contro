#ifndef BME680_READING_H
#define BME680_READING_H

#include <Arduino.h>

struct BME680Reading
{
    float temperatureC = NAN;
    float humidityPct = NAN;
    float pressurehPa = NAN;
    float gasResistanceOhm = NAN;
    uint32_t timestampMs = 0;
    bool valid = false;
};

#endif
