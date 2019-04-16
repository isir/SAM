#ifndef COMPENSATIONOPTITRACK_H
#define COMPENSATIONOPTITRACK_H

#include "algorithms/lawopti.h"
#include "peripherals/XIMU.h"
#include "peripherals/adafruit_ads1115.h"
#include "peripherals/buzzer.h"
#include "peripherals/helpers/osmerelbow.h"
#include "peripherals/helpers/pronosupination.h"
#include "peripherals/touch_bionics/touch_bionics_hand.h"
#include "ui/consolemenu.h"
#include "utils/optilistener.h"
#include "utils/settings.h"
#include <QFile>
#include <QTime>
#include <QUdpSocket>

class CompensationOptitrack : public QObject {
    Q_OBJECT
public:
    explicit CompensationOptitrack(QObject* parent = nullptr);
    ~CompensationOptitrack();

    ConsoleMenu& menu() { return _menu; }

    void start(QString filename);
    void stop();
    void zero();
    void display_parameters();
    void display_lengths();
    void displayArduino();

private:
    OsmerElbow& _osmer;
    PronoSupination& _pronosup;
    OptiListener& _optitrack;
    TouchBionicsHand& _hand;
    QUdpSocket _receiver;
    QUdpSocket _receiverArduino;
    Adafruit_ADS1115 _adc;
    Buzzer _buzzer;

    XIMU _imu_bras;
    XIMU _imu_tronc;

    QTime _abs_time;
    QTime _time;
    int _previous_elapsed;

    QFile _file;
    bool _need_to_write_header;
    Settings _settings;
    LawOpti _lawopti;
    unsigned int _cnt;
    unsigned int _ind;
    unsigned int _infoSent;

    ConsoleMenu _menu;

    int _Lt;
    double _Lua;
    double _Lfa;
    double _l;
    int _lsh;
    int _lambdaW, _lambda;
    double _thresholdW, _threshold;
    int _pinArduino;
    int _pin_up;
    int _pin_down;

private slots:
    void on_activated();
    void on_new_data_compensation(optitrack_data_t data);
    void on_new_data_vol(optitrack_data_t data);
    void read_optiData(optitrack_data_t data);
    void on_def();
    void listenArduino();
};

#endif // COMPENSATIONOPTITRACK_H
