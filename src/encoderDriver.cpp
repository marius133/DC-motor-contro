#include "encoderDriver.h"

#include <cmath>

EncoderDriver::EncoderDriver(Encoder& enc, const config& cfg)
    : _enc(enc), _cfg(cfg)
{
}

void EncoderDriver::zero(bool also_write_encoder)
{
    _count_now = 0;
    _count_prev = 0;
    _delta = 0;
    _speed_cps = 0.0f;

    if (also_write_encoder)
    {
        _enc.write(0);
    }
}

void EncoderDriver::update(uint32_t dt_us)
{
    _count_prev = _count_now;
    _count_now = readEncoderCounts_();
    _delta = _count_now - _count_prev;

    if (dt_us == 0)
    {
        return;
    }

    const float dt_s = static_cast<float>(dt_us) * 1.0e-6f;
    const float rawSpeed = static_cast<float>(_delta) / dt_s;
    const float alpha = constrain(_cfg.l_pass_alpha, 0.0f, 1.0f);

    _speed_cps = alpha * rawSpeed + (1.0f - alpha) * _speed_cps;
}

int32_t EncoderDriver::count() const
{
    return _count_now;
}

int32_t EncoderDriver::delta() const
{
    return _delta;
}

float EncoderDriver::speed_counts_per_sec() const
{
    return _speed_cps;
}

int32_t EncoderDriver::readEncoderCounts_()
{
    const long rawCount = _enc.read();
    if (_cfg.invert)
    {
        return static_cast<int32_t>(-rawCount);
    }

    return static_cast<int32_t>(rawCount);
}
