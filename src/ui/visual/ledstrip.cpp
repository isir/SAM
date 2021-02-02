#include "ledstrip.h"
#include "utils/log/log.h"
#include <bcm2835.h>

static const uint8_t led_value = 50;

LedStrip::color LedStrip::white(led_value, led_value, led_value, 1);
LedStrip::color LedStrip::red(led_value, 0, 0, 1);
LedStrip::color LedStrip::green(0, led_value, 0, 1);
LedStrip::color LedStrip::blue(0, 0, led_value, 1);
LedStrip::color LedStrip::none(0, 0, 0, 0);

LedStrip::LedStrip()
{
    if (bcm2835_spi_begin() < 0) {
        critical() << "bcm2835_spi_begin failed";
    }
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_512);
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
}

LedStrip::~LedStrip()
{
    bcm2835_spi_end();
}

void LedStrip::set(std::vector<color> colors)
{
    send_opening_bytes();
    for (color c : colors) {
        send_color_bytes(c);
    }
    send_closing_bytes();
}

void LedStrip::set(color c, unsigned int n)
{
    send_opening_bytes();
    for (unsigned int i = 0; i < n; ++i) {
        send_color_bytes(c);
    }
    send_closing_bytes();
}

void LedStrip::send_opening_bytes()
{
    char buf[4] = { 0x00, 0x00, 0x00, 0x00 };
    bcm2835_spi_writenb(reinterpret_cast<char*>(&buf), 4);
}

void LedStrip::send_closing_bytes()
{
    char buf[4] = { 0xff, 0xff, 0xff, 0xff };
    bcm2835_spi_writenb(reinterpret_cast<char*>(&buf), 4);
}

void LedStrip::send_color_bytes(color c)
{
    char led_frame[4];
    led_frame[0] = 0b11100000 | (0b00011111 & c.brightness);
    led_frame[1] = c.b;
    led_frame[2] = c.g;
    led_frame[3] = c.r;

    bcm2835_spi_writenb(reinterpret_cast<char*>(&led_frame), 4);
}
