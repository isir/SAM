#ifndef BUZZER_H
#define BUZZER_H

#include "components/internal/gpio/gpio.h"
#include "utils/threaded_loop.h"

/**
 * \brief The Buzzer class handles the buzzer on the prosthesis. Through it, you can beep the buzzer in a thread with different patterns.
 */
class Buzzer : public ThreadedLoop {
public:
    enum BUZZ_TYPE {
        NO_BUZZ,
        STANDARD_BUZZ,
        DOUBLE_BUZZ,
        TRIPLE_BUZZ,
        SHORT_BUZZ,
        ERROR_BUZZ
    };

    struct BuzzerConfig {
        int n_buzzes;
        int n_pulses;
        int half_period_us;
        int time_between_buzzes_us;
    };

    Buzzer(int pin);
    ~Buzzer() override;

    void makeNoise(BUZZ_TYPE buzz_type = STANDARD_BUZZ, int freq = 10000);

private:
    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;

    GPIO _gpio;
    BuzzerConfig _cfg;
};

#endif // BUZZER_H
