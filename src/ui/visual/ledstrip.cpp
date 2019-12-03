#include "ledstrip.h"
#include "utils/log/log.h"
#include "wiringPiSPI.h"

static const uint8_t led_value = 50;

LedStrip::color LedStrip::white(led_value, led_value, led_value, 1);
LedStrip::color LedStrip::red(led_value, 0, 0, 1);
LedStrip::color LedStrip::red_bright(3*led_value, 0, 0, 1);
LedStrip::color LedStrip::green(0, led_value, 0, 1);
LedStrip::color LedStrip::blue(0, 0, led_value, 1);
LedStrip::color LedStrip::none(0, 0, 0, 0);

LedStrip::LedStrip()
{
    if (wiringPiSPISetup(0, 500000) < 0) {
        critical() << "wiringPiSPISetup failed";
    }
}

LedStrip::~LedStrip()
{
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
    uint8_t buf[4] = { 0x00, 0x00, 0x00, 0x00 };
    wiringPiSPIDataRW(0, reinterpret_cast<unsigned char*>(&buf), 4);
}

void LedStrip::send_closing_bytes()
{
    uint8_t buf[4] = { 0xff, 0xff, 0xff, 0xff };
    wiringPiSPIDataRW(0, reinterpret_cast<unsigned char*>(&buf), 4);
}

void LedStrip::send_color_bytes(color c)
{
    uint8_t led_frame[4];
    led_frame[0] = 0b11100000 | (0b00011111 & c.brightness);
    led_frame[1] = c.b;
    led_frame[2] = c.g;
    led_frame[3] = c.r;

    wiringPiSPIDataRW(0, led_frame, 4);
}
