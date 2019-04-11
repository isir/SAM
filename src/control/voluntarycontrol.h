#ifndef VOLUNTARYCONTROL_H
#define VOLUNTARYCONTROL_H

#include "basiccontroller.h"
#include "peripherals/helpers/osmerelbow.h"
#include "peripherals/helpers/pronosupination.h"
#include "utils/optilistener.h"
#include "utils/settings.h"
#include <QFile>

class VoluntaryControl : public BasicController {
    Q_OBJECT
public:
    explicit VoluntaryControl();
    ~VoluntaryControl();

private:
    bool setup();
    void loop(double dt, double time);
    void cleanup();

    QFile _file;
    bool _need_to_write_header;
    Settings _settings;
    OsmerElbow& _osmer;
    PronoSupination& _pronosup;
    OptiListener& _optilistener;

    int _pin_up;
    int _pin_down;
};

#endif // VOLUNTARYCONTROL_H
