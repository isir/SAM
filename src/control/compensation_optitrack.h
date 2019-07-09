#ifndef COMPENSATIONOPTITRACK_H
#define COMPENSATIONOPTITRACK_H

#include "algorithms/lawopti.h"
#include "ui/menu_user.h"
#include "utils/opti_listener.h"
#include "utils/sam.h"
#include "utils/settings.h"
#include <QFile>
#include <QTime>
#include <QUdpSocket>

class CompensationOptitrack : public QObject, public MenuUser {
    Q_OBJECT
public:
    explicit CompensationOptitrack(std::shared_ptr<SAM::Components> robot);
    ~CompensationOptitrack();

    void start(QString filename);
    void stop();
    void zero();
    void display_parameters();
    void display_lengths();
    void displayArduino();

private:
    std::shared_ptr<SAM::Components> _robot;
    QUdpSocket _receiver;
    QUdpSocket _receiverArduino;

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
