#include "ledstrip.h"
#include "wiringPiSPI.h"
#include <QDebug>

static const quint8 led_value = 50;

LedStrip::color LedStrip::white(led_value, led_value, led_value, 1);
LedStrip::color LedStrip::red(led_value, 0, 0, 1);
LedStrip::color LedStrip::green(0, led_value, 0, 1);
LedStrip::color LedStrip::blue(0, 0, led_value, 1);
LedStrip::color LedStrip::none(0, 0, 0, 0);

LedStrip& LedStrip::instance()
{
    static LedStrip ls;
    return ls;
}

LedStrip::LedStrip()
    : QObject()
{
    if (wiringPiSPISetup(0, 500000) < 0) {
        qCritical() << "wiringPiSPISetup failed";
    }
    QObject::connect(&_sequence_timer, &QTimer::timeout, this, &LedStrip::sequence_callback);
}

LedStrip::~LedStrip()
{
}

void LedStrip::set(QVector<color> colors)
{
    //stop_sequence();
    send_opening_bytes();
    foreach (color c, colors) {
        send_color_bytes(c);
    }
    send_closing_bytes();
}

void LedStrip::set(color c, unsigned int n)
{
    //stop_sequence();
    send_opening_bytes();
    for (unsigned int i = 0; i < n; ++i) {
        send_color_bytes(c);
    }
    send_closing_bytes();
}

void LedStrip::play_sequence(QVector<QVector<color>> sequence, unsigned int period_ms)
{
    stop_sequence();
    _sequence = sequence;
    _sequence_idx = 0;
    _sequence_timer.setSingleShot(false);
    _sequence_timer.start(period_ms);
}

void LedStrip::stop_sequence()
{
    _sequence_timer.stop();
}

void LedStrip::send_opening_bytes()
{
    quint8 buf[4] = { 0x00, 0x00, 0x00, 0x00 };
    wiringPiSPIDataRW(0, reinterpret_cast<unsigned char*>(&buf), 4);
}

void LedStrip::send_closing_bytes()
{
    quint8 buf[4] = { 0xff, 0xff, 0xff, 0xff };
    wiringPiSPIDataRW(0, reinterpret_cast<unsigned char*>(&buf), 4);
}

void LedStrip::send_color_bytes(color c)
{
    quint8 led_frame[4];
    led_frame[0] = 0b11100000 | (0b00011111 & c.brightness);
    led_frame[1] = c.b;
    led_frame[2] = c.g;
    led_frame[3] = c.r;

    wiringPiSPIDataRW(0, led_frame, 4);
}

void LedStrip::sequence_callback()
{
    if (_sequence_idx < _sequence.size()) {
        set(_sequence.at(_sequence_idx));
        ++_sequence_idx;
    } else {
        _sequence_idx = 0;
    }
}
