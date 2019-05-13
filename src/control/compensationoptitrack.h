#ifndef COMPENSATIONOPTITRACK_H
#define COMPENSATIONOPTITRACK_H

#include "algorithms/lawopti.h"
#include "ui/consolemenu.h"
#include "utils/optilistener.h"
#include "utils/sam.h"
#include "utils/settings.h"
#include <QFile>
#include <QTime>
#include <QUdpSocket>

class CompensationOptitrack : public QObject {
    Q_OBJECT
public:
    explicit CompensationOptitrack(SAM::Components robot, std::shared_ptr<QMqttClient> mqtt);
    ~CompensationOptitrack();

    ConsoleMenu& menu() { return _menu; }

    void start(QString filename);
    void stop();
    void zero();
    void display_parameters();
    void display_lengths();
    void displayArduino();

private:
    SAM::Components _robot;
    QUdpSocket _receiver;
    QUdpSocket _receiverArduino;

    QTime _abs_time;
    QTime _time;
    int _previous_elapsed;
    double _old_time;

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
    void tareIMU();
};

#endif // COMPENSATIONOPTITRACK_H
