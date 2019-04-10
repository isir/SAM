#ifndef BUZZER_H
#define BUZZER_H

#include "wiringPi.h"
#include <QObject>
#include <QThread>

namespace BuzzerConfig {
enum BUZZ_TYPE {
    NO_BUZZ,
    STANDARD_BUZZ,
    DOUBLE_BUZZ,
    TRIPLE_BUZZ,
    SHORT_BUZZ,
    ERROR_BUZZ
};

typedef struct {
    int n_buzzes;
    int pin;
    int n_pulses;
    int half_period_us;
    int time_between_buzzes_us;
} buzzer_config_t;
}

Q_DECLARE_METATYPE(BuzzerConfig::buzzer_config_t)

class BuzzerWorker : public QObject {
    Q_OBJECT

public slots:
    void doWork(BuzzerConfig::buzzer_config_t config)
    {
        for (; config.n_buzzes > 0; --config.n_buzzes) {
            for (int i = 0; i < config.n_pulses; i++) {
                digitalWrite(config.pin, 1);
                QThread::usleep(config.half_period_us);
                digitalWrite(config.pin, 0);
                QThread::usleep(config.half_period_us);
            }
            if (config.n_buzzes > 0)
                QThread::usleep(config.time_between_buzzes_us);
        }
    }
};

/**
 * \brief The Buzzer class handles the buzzer on the prosthesis. Through it, you can beep the buzzer in a thread with different patterns.
 */
class Buzzer : public QObject {
    Q_OBJECT
public:
    Buzzer(int pin);
    ~Buzzer();

public slots:
    void makeNoise(BuzzerConfig::BUZZ_TYPE buzz_type = BuzzerConfig::STANDARD_BUZZ, int freq = 10000);

protected:
    int _pin;
    QThread _thread;
    BuzzerWorker _worker;

signals:
    void buzz(BuzzerConfig::buzzer_config_t config);
};

#endif // BUZZER_H
