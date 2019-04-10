#ifndef LEDSTRIP_H
#define LEDSTRIP_H

#include <QObject>
#include <QTimer>
#include <QVector>
#include <QtGlobal>

class LedStrip : public QObject {
    Q_OBJECT
public:
    struct color {
        color()
            : r(0)
            , g(0)
            , b(0)
            , brightness(0)
        {
        }
        color(quint8 r_, quint8 g_, quint8 b_, quint8 brightness_)
            : r(r_)
            , g(g_)
            , b(b_)
            , brightness(brightness_)
        {
        }

        quint8 r;
        quint8 g;
        quint8 b;
        quint8 brightness;
    };

    static color white, red, green, blue, none;

    static LedStrip& instance();
    ~LedStrip();
    void set(QVector<color> colors);
    void set(color c, unsigned int n);

    void play_sequence(QVector<QVector<color>> sequence, unsigned int period_ms);
    void stop_sequence();

private:
    LedStrip();

    void send_opening_bytes();
    void send_closing_bytes();
    void send_color_bytes(color c);

    QTimer _sequence_timer;
    QVector<QVector<color>> _sequence;
    int _sequence_idx;

private slots:
    void sequence_callback();
};

#endif // LEDSTRIP_H
