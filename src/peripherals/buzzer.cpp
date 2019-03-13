#include "buzzer.h"
#include <QDebug>

Buzzer::Buzzer(int pin) :  QObject(), _pin(pin)
{
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);

    qRegisterMetaType<BuzzerConfig::buzzer_config_t>("BuzzerConfig::buzzer_config_t");
    _worker.moveToThread(&_thread);
    QObject::connect(this,&Buzzer::buzz,&_worker,&BuzzerWorker::doWork);
    _thread.start();
}

Buzzer::~Buzzer() {
    _thread.quit();
    _thread.wait();
}

void Buzzer::makeNoise(BuzzerConfig::BUZZ_TYPE buzz_type, int freq) {
    BuzzerConfig::buzzer_config_t config;
    config.pin = _pin;
    config.n_pulses = 500;
    config.half_period_us = static_cast<int>(1000000. / freq);
    config.time_between_buzzes_us = 200000;
    config.n_buzzes = 1;

    switch(buzz_type) {
    case BuzzerConfig::NO_BUZZ:
        return;
    case BuzzerConfig::STANDARD_BUZZ:
        break;
    case BuzzerConfig::DOUBLE_BUZZ:
        config.n_buzzes = 2;
        break;
    case BuzzerConfig::TRIPLE_BUZZ:
        config.n_buzzes = 3;
        break;
    case BuzzerConfig::SHORT_BUZZ:
        config.n_pulses = 200;
        break;
    case BuzzerConfig::ERROR_BUZZ:
        config.n_pulses = 200;
        config.n_buzzes = 15;
        break;
    default:
        break;
    }

    emit buzz(config);
}
