#ifndef GENERAL_FORMULATION_H
#define GENERAL_FORMULATION_H

#include "algorithms/lawjacobian.h"
#include "threaded_loop.h"
#include "utils/opti_listener.h"
#include "utils/sam.h"
#include "utils/settings.h"
#include <QFile>
#include <QTime>
#include <QUdpSocket>

class GeneralFormulation : public ThreadedLoop {
    Q_OBJECT
public:
    explicit GeneralFormulation(std::shared_ptr<SAM::Components> robot);
    ~GeneralFormulation();

private:
    void tare_IMU();
    void receiveData();
    void displayPin();
    bool setup();
    void loop(double dt, double time);
    void cleanup();

    std::shared_ptr<SAM::Components> _robot;
    QUdpSocket _receiver;
    QFile _file;
    bool _need_to_write_header;
    Settings _settings;
    int _cnt;
    QTime _time;

    LawJacobian _lawJ;
    int _Lt;
    double _Lua;
    double _Lfa;
    double _lhand;
    double l[nbLinks];
    int _lambdaW, _lambda;
    int _pin_up;
    int _pin_down;
    double theta[nbLinks];
    double _threshold[nbLinks];
};

#endif // GENERAL_FORMULATION_H
