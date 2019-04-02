#ifndef VOLUNTARYCONTROL_H
#define VOLUNTARYCONTROL_H

<<<<<<< HEAD
#include <QSettings>
#include <QFile>
=======
>>>>>>> 33eeae6fb5852c7cfbd80fb2df1a053c3385cb5a
#include "basiccontroller.h"
#include "peripherals/helpers/osmerelbow.h"
#include <QSettings>

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
    QSettings _settings;
    OsmerElbow& _osmer;

    int _pin_up;
    int _pin_down;
};

#endif // VOLUNTARYCONTROL_H
