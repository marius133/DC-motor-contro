#include "CsvLogger.h"

#include <cmath>

bool CsvLogger::begin(uint8_t chipSelect, const char* filename)
{
    chipSelect_ = chipSelect;
    filename_ = filename;
    initialized_ = SD.begin(chipSelect_);
    return initialized_;
}

bool CsvLogger::append(const LogRecord& record)
{
    if (!initialized_)
    {
        return false;
    }

    File file = SD.open(filename_, FILE_WRITE);
    if (!file)
    {
        return false;
    }

    // The header is written lazily so a fresh card can be inserted without an
    // additional setup step.
    if (file.size() == 0 && !writeHeader_(file))
    {
        file.close();
        return false;
    }

    file.print(record.deviceTimestampMs);
    file.print(',');
    printDateTime_(file, record.gps);
    file.print(',');
    printDoubleOrBlank_(file, record.gps.latitudeDeg, 7);
    file.print(',');
    printDoubleOrBlank_(file, record.gps.longitudeDeg, 7);
    file.print(',');
    printFloatOrBlank_(file, record.gps.altitudeM, 2);
    file.print(',');
    printUintOrBlank_(file, record.gps.satellites, record.gps.fix);
    file.print(',');
    printFloatOrBlank_(file, record.gps.hdop, 2);
    file.print(',');
    file.print(record.gps.fix ? 1 : 0);
    file.print(',');
    printFloatOrBlank_(file, record.environment.temperatureC, 2);
    file.print(',');
    printFloatOrBlank_(file, record.environment.humidityPct, 2);
    file.print(',');
    printFloatOrBlank_(file, record.environment.pressurehPa, 2);
    file.print(',');
    printFloatOrBlank_(file, record.environment.gasResistanceOhm, 2);
    file.println();
    file.flush();
    file.close();
    return true;
}

bool CsvLogger::isInitialized() const
{
    return initialized_;
}

const char* CsvLogger::filename() const
{
    return filename_;
}

bool CsvLogger::writeHeader_(File& file)
{
    file.println("device_ms,gps_utc,latitude_deg,longitude_deg,altitude_m,satellites,hdop,gps_fix,temperature_c,humidity_pct,pressure_hpa,gas_resistance_ohm");
    return true;
}

void CsvLogger::printDateTime_(File& file, const GPSReading& gps)
{
    if (!gps.valid || gps.year == 0)
    {
        return;
    }

    char buffer[32];
    snprintf(buffer,
             sizeof(buffer),
             "%04u-%02u-%02uT%02u:%02u:%02uZ",
             gps.year,
             gps.month,
             gps.day,
             gps.hour,
             gps.minute,
             gps.second);
    file.print(buffer);
}

void CsvLogger::printDoubleOrBlank_(File& file, double value, uint8_t precision)
{
    // Blank cells are easier to handle downstream than sentinel values.
    if (isnan(value))
    {
        return;
    }

    file.print(value, precision);
}

void CsvLogger::printFloatOrBlank_(File& file, float value, uint8_t precision)
{
    if (isnan(value))
    {
        return;
    }

    file.print(value, precision);
}

void CsvLogger::printUintOrBlank_(File& file, uint32_t value, bool valid)
{
    if (!valid)
    {
        return;
    }

    file.print(value);
}
