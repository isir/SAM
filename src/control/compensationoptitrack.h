#ifndef COMPENSATIONOPTITRACK_H
#define COMPENSATIONOPTITRACK_H

#include <QSettings>
#include <QUdpSocket>
#include <QFile>
#include <QTime>
#include "ui/consolemenu.h"
#include "peripherals/helpers/osmerelbow.h"
#include "peripherals/helpers/pronosupination.h"
#include "peripherals/adafruit_ads1115.h"
#include "peripherals/XIMU.h"
#include "utils/optilistener.h"
#include "algorithms/lawopti.h"

class CompensationOptitrack : public QObject
{
    Q_OBJECT
public:
    explicit CompensationOptitrack(QObject *parent = nullptr);
    ~CompensationOptitrack();

    ConsoleMenu& menu() { return _menu; }

    void start(QString filename);
    void stop();
    void zero();
    void display_parameters();
    void display_optiData();

private:
    OsmerElbow& _osmer;
    PronoSupination& _pronosup;
    OptiListener _optitrack;
    QUdpSocket _receiver;
    Adafruit_ADS1115 _adc;

    XIMU _imu_bras;
    XIMU _imu_tronc;

    QTime _abs_time;
    QTime _time;
    int _previous_elapsed;

    QFile _file;
    bool _need_to_write_header;
    QSettings _settings;
    LawOpti _lawopti;
    unsigned int _cnt;

    ConsoleMenu _menu;

    int _Lt;
    double _Lua;
    double _Lfa;
    int _lsh;
    int _lambda, _lambdaW;
    double _threshold, _thresholdW;

private slots:
    void on_activated();
    void on_new_data(optitrack_data_t data);
    void read_optiData(optitrack_data_t data);
    void on_def();
};

#endif // COMPENSATIONOPTITRACK_H
