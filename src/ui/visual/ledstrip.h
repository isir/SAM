#ifndef LEDSTRIP_H
#define LEDSTRIP_H

#include <stdint.h>
#include <vector>

class LedStrip {
public:
    struct color {
        color()
            : r(0)
            , g(0)
            , b(0)
            , brightness(0)
        {
        }
        color(uint8_t r_, uint8_t g_, uint8_t b_, uint8_t brightness_)
            : r(r_)
            , g(g_)
            , b(b_)
            , brightness(brightness_)
        {
        }

        uint8_t r;
        uint8_t g;
        uint8_t b;
        uint8_t brightness;
    };

    static color white, red, green, blue, none;

    LedStrip();
    ~LedStrip();
    void set(std::vector<color> colors);
    void set(color c, unsigned int n);

private:
    void send_opening_bytes();
    void send_closing_bytes();
    void send_color_bytes(color c);
};

#endif // LEDSTRIP_H
