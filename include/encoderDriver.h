#pragma once

#include <Arduino.h>
#include <Encoder.h>

class EncoderDriver
{
public:
// Configuration struct for the EncoderDriver class, containing parameters for counts per revolution, inversion of counts, and low-pass filter alpha for speed calculation
    struct config
    {
        int32_t counts_per_rev;
        bool invert = false;
        float l_pass_alpha = 0.2f;
    };

    // Constructor for EncoderDriver class with reference to an Encoder object and a configuration struct
    EncoderDriver(Encoder &enc, const config &cfg);

    // Method to zero the encoder counts, with an option to also write the zeroed value back to the encoder
    void zero(bool also_write_encoder = true);
    void update(uint32_t dt_us);
    int32_t count() const;
    int32_t delta() const;
    float speed_counts_per_sec() const;

private:
    Encoder &_enc;
    config _cfg;
    int32_t _count_now = 0;
    int32_t _count_prev = 0;
    int32_t _delta = 0;
    float _speed_cps = 0.0f;
    int32_t readEncoderCounts_();
};
