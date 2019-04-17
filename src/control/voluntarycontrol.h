#ifndef VOLUNTARYCONTROL_H
#define VOLUNTARYCONTROL_H

#include "basiccontroller.h"
#include "utils/optilistener.h"
#include "utils/sam.h"
#include "utils/settings.h"
#include <QFile>

class VoluntaryControl : public BasicController {
    Q_OBJECT
public:
    explicit VoluntaryControl(SAM::Components robot, std::shared_ptr<QMqttClient> mqtt);
    ~VoluntaryControl();

private:
    bool setup();
    void loop(double dt, double time);
    void cleanup();

    SAM::Components _robot;
    QFile _file;
    bool _need_to_write_header;
    Settings _settings;
    OptiListener& _optilistener;

    int _pin_up;
    int _pin_down;
};

#endif // VOLUNTARYCONTROL_H
