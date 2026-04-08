#include "BME680Sensor.h"

#include <new>

#if defined(ARDUINO_TEENSY_MICROMOD)
#define BME680_HAS_WIRE3 1
#else
#define BME680_HAS_WIRE3 0
#endif

BME680Sensor::BME680Sensor()
{
    rebuildSensor_(Wire);
}

BME680Sensor::~BME680Sensor()
{
    delete sensor_;
    sensor_ = nullptr;
}

bool BME680Sensor::begin()
{
    return begin(Config{});
}

bool BME680Sensor::begin(const Config& config)
{
    cfg_ = config;
    resetReading_();
    detectedBus_ = Bus::Auto;
    detectedAddress_ = 0;

    const uint8_t secondaryAddress = (cfg_.address == 0x76) ? 0x77 : 0x76;
    const bool trySecondaryAddress = cfg_.autoDetectAddress && secondaryAddress != cfg_.address;

    const auto tryBus = [&](TwoWire& wire, Bus bus) -> bool
    {
        if (tryBeginOnBusAtAddress_(wire, bus, cfg_.address))
        {
            return true;
        }

        if (trySecondaryAddress && tryBeginOnBusAtAddress_(wire, bus, secondaryAddress))
        {
            return true;
        }

        return false;
    };

    if (cfg_.bus == Bus::Auto)
    {
        initialized_ = tryBus(Wire, Bus::Wire0) ||
                       tryBus(Wire1, Bus::Wire1) ||
                       tryBus(Wire2, Bus::Wire2);
#if BME680_HAS_WIRE3
        if (!initialized_)
        {
            initialized_ = tryBus(Wire3, Bus::Wire3);
        }
#endif
    }
    else
    {
        switch (cfg_.bus)
        {
            case Bus::Wire0:
                initialized_ = tryBus(Wire, Bus::Wire0);
                break;
            case Bus::Wire1:
                initialized_ = tryBus(Wire1, Bus::Wire1);
                break;
            case Bus::Wire2:
                initialized_ = tryBus(Wire2, Bus::Wire2);
                break;
            case Bus::Wire3:
#if BME680_HAS_WIRE3
                initialized_ = tryBus(Wire3, Bus::Wire3);
#else
                initialized_ = false;
#endif
                break;
            case Bus::Auto:
            default:
                initialized_ = false;
                break;
        }
    }

    if (!initialized_)
    {
        rebuildSensor_(Wire);
        status_ = Status::NotInitialized;
        return false;
    }

    applyConfig_();
    status_ = Status::Ready;
    return true;
}

bool BME680Sensor::read()
{
    if (!initialized_)
    {
        resetReading_();
        status_ = Status::NotInitialized;
        return false;
    }

    if (sensor_ == nullptr || !sensor_->performReading())
    {
        resetReading_();
        status_ = Status::ReadFailed;
        return false;
    }

    reading_.temperatureC = sensor_->temperature + cfg_.temperatureOffsetC;
    reading_.humidityPct = sensor_->humidity;
    reading_.pressurehPa = sensor_->pressure / 100.0f;
    reading_.gasResistanceOhm = static_cast<float>(sensor_->gas_resistance);
    reading_.timestampMs = millis();
    reading_.valid = true;
    status_ = Status::Ready;
    return true;
}

bool BME680Sensor::isInitialized() const
{
    return initialized_;
}

bool BME680Sensor::isDataValid() const
{
    return reading_.valid;
}

BME680Sensor::Status BME680Sensor::status() const
{
    return status_;
}

BME680Sensor::Bus BME680Sensor::detectedBus() const
{
    return detectedBus_;
}

const char* BME680Sensor::detectedBusName() const
{
    switch (detectedBus_)
    {
        case Bus::Wire0: return "Wire";
        case Bus::Wire1: return "Wire1";
        case Bus::Wire2: return "Wire2";
        case Bus::Wire3: return "Wire3";
        case Bus::Auto:
        default: return "unknown";
    }
}

uint8_t BME680Sensor::detectedAddress() const
{
    return detectedAddress_;
}

const BME680Reading& BME680Sensor::getReading() const
{
    return reading_;
}

bool BME680Sensor::tryBeginOnBusAtAddress_(TwoWire& wire, Bus bus, uint8_t address)
{
    rebuildSensor_(wire);
    wire.begin();
    wire.setClock(400000);

    if (sensor_ == nullptr || !sensor_->begin(address))
    {
        return false;
    }

    detectedBus_ = bus;
    detectedAddress_ = address;
    return true;
}

void BME680Sensor::rebuildSensor_(TwoWire& wire)
{
    delete sensor_;
    sensor_ = new Adafruit_BME680(&wire);
}

void BME680Sensor::applyConfig_()
{
    if (sensor_ == nullptr)
    {
        return;
    }

    sensor_->setTemperatureOversampling(cfg_.temperatureOversampling);
    sensor_->setHumidityOversampling(cfg_.humidityOversampling);
    sensor_->setPressureOversampling(cfg_.pressureOversampling);
    sensor_->setIIRFilterSize(cfg_.filterSize);
    sensor_->setGasHeater(cfg_.heaterTempC, cfg_.heaterTimeMs);
}

void BME680Sensor::resetReading_()
{
    reading_ = {};
}
