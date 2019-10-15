#include "gpio.h"
#include "wiringPi.h"

GPIO::GPIO(int pin, Direction dir, Pull pull)
    : _pin(pin)
    , _dir(dir)
    , _pull(pull)
{
    switch (dir) {
    case DIR_INPUT:
        pinMode(_pin, INPUT);
        break;
    case DIR_OUTPUT:
        pinMode(_pin, OUTPUT);
        break;
    }

    switch (pull) {
    case PULL_UP:
        pullUpDnControl(_pin, PUD_UP);
        break;
    case PULL_DOWN:
        pullUpDnControl(_pin, PUD_DOWN);
        break;
    case PULL_NONE:
        pullUpDnControl(_pin, PUD_OFF);
        break;
    }
}

GPIO& GPIO::operator=(int v)
{
    if (_dir == DIR_OUTPUT) {
        digitalWrite(_pin, v);
    }
    return *this;
}

GPIO& GPIO::operator=(bool v)
{
    if (_dir == DIR_OUTPUT) {
        digitalWrite(_pin, v ? 1 : 0);
    }
    return *this;
}

GPIO::operator int()
{
    if (_dir == DIR_INPUT) {
        return digitalRead(_pin);
    } else {
        return 0;
    }
}

GPIO::operator bool()
{
    if (_dir == DIR_INPUT) {
        return digitalRead(_pin) == HIGH;
    } else {
        return 0;
    }
}
