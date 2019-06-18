#ifndef VOLUNTARYCONTROL_H
#define VOLUNTARYCONTROL_H

#include "basic_controller.h"
#include "utils/opti_listener.h"
#include "utils/sam.h"
#include "utils/settings.h"
#include <QFile>

class VoluntaryControl : public BasicController {
    Q_OBJECT
public:
    explicit VoluntaryControl(std::shared_ptr<SAM::Components> robot);
    ~VoluntaryControl();

private:
    bool setup();
    void loop(double dt, double time);
    void cleanup();

    std::shared_ptr<SAM::Components> _robot;
    QFile _file;
    bool _need_to_write_header;
    Settings _settings;

    int _pin_up;
    int _pin_down;
};

#endif // VOLUNTARYCONTROL_H
