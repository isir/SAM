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
    bool setup();
    void loop(double dt, double time);
    void cleanup();

    SAM::Components _robot;
    QFile _file;
    bool _need_to_write_header;
    Settings _settings;
    int _cnt;
    QTime _time;
    LawIMU _lawimu;
};

#endif // COMPENSATION_IMU_H
