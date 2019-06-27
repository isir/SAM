#ifndef COMPENSATIONIMU_H
#define COMPENSATIONIMU_H

#include "basic_controller.h"
#include "control/algorithms/lawimu.h"
#include "utils/opti_listener.h"
#include "utils/sam.h"
#include "utils/settings.h"
#include <QFile>
#include <QTime>

class CompensationIMU : public BasicController {
    Q_OBJECT
public:
    explicit CompensationIMU(SAM::Components robot, std::shared_ptr<QMqttClient> mqtt);
    ~CompensationIMU();

private:
    void tareIMU();
    bool setup();
    void loop(double dt, double time);
    void cleanup();

    SAM::Components _robot;
    QUdpSocket _receiver;
    QFile _file;
    bool _need_to_write_header;
    Settings _settings;
    int _cnt;
    QTime _time;
    LawIMU _lawimu;

    int _Lt;
    double _Lua;
    double _Lfa;
    double _l;
    int _lambdaW, _lambda;
    double _thresholdW, _threshold;
};

#endif // COMPENSATION_IMU_H
