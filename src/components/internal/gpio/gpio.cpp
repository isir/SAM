#include "gpio.h"
#include <bcm2835.h>

GPIO::GPIO(int pin, Direction dir, Pull pull)
    : _pin(pin)
    , _dir(dir)
    , _pull(pull)
{
    switch (dir) {
    case DIR_INPUT:
        bcm2835_gpio_fsel(_pin, BCM2835_GPIO_FSEL_INPT);
        break;
    case DIR_OUTPUT:
        bcm2835_gpio_fsel(_pin, BCM2835_GPIO_FSEL_OUTP);
        break;
    }

    switch (pull) {
    case PULL_UP:
        bcm2835_gpio_set_pud(_pin, BCM2835_GPIO_PUD_UP);
        break;
    case PULL_DOWN:
        bcm2835_gpio_set_pud(_pin, BCM2835_GPIO_PUD_DOWN);
        break;
    case PULL_NONE:
        bcm2835_gpio_set_pud(_pin, BCM2835_GPIO_PUD_OFF);
        break;
    }
}

GPIO& GPIO::operator=(int v)
{
    if (_dir == DIR_OUTPUT) {
        bcm2835_gpio_write(_pin, v);
    }
    return *this;
}

GPIO& GPIO::operator=(bool v)
{
    if (_dir == DIR_OUTPUT) {
        bcm2835_gpio_write(_pin, v ? HIGH : LOW);
    }
    return *this;
}

GPIO::operator int()
{
    if (_dir == DIR_INPUT) {
        return bcm2835_gpio_lev(_pin);
    } else {
        return 0;
    }
}

GPIO::operator bool()
{
    if (_dir == DIR_INPUT) {
        return bcm2835_gpio_lev(_pin) == HIGH;
    } else {
        return 0;
    }
}
