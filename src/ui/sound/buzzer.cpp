#include "buzzer.h"

Buzzer::Buzzer(int pin)
    : ThreadedLoop("buzzer", 0.)
    , _pin(pin)
{
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
}

Buzzer::~Buzzer()
{
    stop_and_join();
}

bool Buzzer::setup()
{
    return true;
}

void Buzzer::loop(double, clock::time_point)
{
    for (; _cfg.n_buzzes > 0; --_cfg.n_buzzes) {
        for (int i = 0; i < _cfg.n_pulses; i++) {
            digitalWrite(_cfg.pin, 1);
            std::this_thread::sleep_for(std::chrono::microseconds(_cfg.half_period_us));
            digitalWrite(_cfg.pin, 0);
            std::this_thread::sleep_for(std::chrono::microseconds(_cfg.half_period_us));
        }
        if (_cfg.n_buzzes > 0)
            std::this_thread::sleep_for(std::chrono::microseconds(_cfg.time_between_buzzes_us));
    }
    stop();
}

void Buzzer::cleanup()
{
}

void Buzzer::makeNoise(BUZZ_TYPE buzz_type, int freq)
{
    if (_thread.joinable()) {
        _thread.join();
    }

    _cfg.pin = _pin;
    _cfg.n_pulses = 500;
    _cfg.half_period_us = static_cast<int>(1000000. / freq);
    _cfg.time_between_buzzes_us = 200000;
    _cfg.n_buzzes = 1;

    switch (buzz_type) {
    case NO_BUZZ:
        return;
    case STANDARD_BUZZ:
        break;
    case DOUBLE_BUZZ:
        _cfg.n_buzzes = 2;
        break;
    case TRIPLE_BUZZ:
        _cfg.n_pulses = 200;
        _cfg.time_between_buzzes_us = 50000;
        _cfg.n_buzzes = 3;
        break;
    case SHORT_BUZZ:
        _cfg.n_pulses = 200;
        break;
    case ERROR_BUZZ:
        _cfg.n_pulses = 200;
        _cfg.n_buzzes = 15;
        break;
    }

    start();
}
