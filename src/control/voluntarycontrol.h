#ifndef VOLUNTARYCONTROL_H
#define VOLUNTARYCONTROL_H

#include <QSettings>
#include <QFile>
#include "basiccontroller.h"
#include "peripherals/helpers/osmerelbow.h"

class VoluntaryControl : public BasicController
{
    Q_OBJECT
public:
    explicit VoluntaryControl();
    ~VoluntaryControl();

private:
    void setup();
    void loop(double dt, double time);
    void cleanup();

    QFile _file;
    bool _need_to_write_header;
    QSettings _settings;
    OsmerElbow& _osmer;

    int _pin_up;
    int _pin_down;
};

#endif // VOLUNTARYCONTROL_H
