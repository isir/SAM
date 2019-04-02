#include "voluntarycontrol.h"
#include "peripherals/roboclaw/factory.h"

<<<<<<< HEAD
#include <QNetworkDatagram>
#include <wiringPi.h>
#include <iostream>
#include "qmath.h"
=======
#include <iostream>
#include <wiringPi.h>
>>>>>>> 33eeae6fb5852c7cfbd80fb2df1a053c3385cb5a

VoluntaryControl::VoluntaryControl()
    : _osmer(OsmerElbow::instance())
{
    _settings.beginGroup("VoluntaryControl");
    _pin_up = _settings.value("pin_up", 24).toInt();
    _pin_down = _settings.value("pin_down", 22).toInt();
    set_period(_settings.value("period", 0.01).toDouble());

    _menu.set_title("Voluntary Control");
    _menu.set_code("vc");
    _menu.addItem(_osmer.menu());

    pullUpDnControl(_pin_up, PUD_UP);
    pullUpDnControl(_pin_down, PUD_UP);
}

VoluntaryControl::~VoluntaryControl()
{
    _osmer.forward(0);
}

bool VoluntaryControl::setup()
{
    _osmer.calibration();
<<<<<<< HEAD
    QString filename = QString("voluntary");


    int cnt = 0;
    QString extension = QString(".txt");
    do {
        ++cnt;
        QString suffix = QString("_") + QString::number(cnt);
        _file.setFileName(filename + suffix + extension);
    } while(_file.exists());

    if(!_file.open(QIODevice::ReadWrite)) {
        qCritical() << "Failed to open" << _file.fileName() << "-" << _file.errorString();
        return;
    }
    _need_to_write_header = true;
=======
    return true;
>>>>>>> 33eeae6fb5852c7cfbd80fb2df1a053c3385cb5a
}

void VoluntaryControl::loop(double, double)
{
    static int prev_pin_up_value = 1, prev_pin_down_value = 1;
    int pin_down_value = digitalRead(_pin_down);
    int pin_up_value = digitalRead(_pin_up);

<<<<<<< HEAD
    double beta = _osmer.angle() * M_PI/180.;

    if(pin_down_value == 0 && prev_pin_down_value == 1) {
=======
    if (pin_down_value == 0 && prev_pin_down_value == 1) {
>>>>>>> 33eeae6fb5852c7cfbd80fb2df1a053c3385cb5a
        _osmer.set_velocity(30);
    } else if (pin_up_value == 0 && prev_pin_up_value == 1) {
        _osmer.set_velocity(-30);
<<<<<<< HEAD
    }
    else if ((pin_down_value == 1 && pin_up_value == 1) && (prev_pin_down_value == 0 || prev_pin_up_value == 0)) {
        _osmer.forward(0);
//        sleep(2);
//        _osmer.set_velocity(0);
=======
    } else if ((pin_down_value == 1 && pin_up_value == 1) && (prev_pin_down_value == 0 || prev_pin_up_value == 0)) {
        _osmer.move_to_angle(_osmer.angle());
>>>>>>> 33eeae6fb5852c7cfbd80fb2df1a053c3385cb5a
    }

    prev_pin_down_value = pin_down_value;
    prev_pin_up_value = pin_up_value;

    if(_need_to_write_header) {
        _file.write("period, btnUp, btnDown, beta");
        _file.write("\r\n");
        _need_to_write_header = false;
    }

    QTextStream ts(&_file);
    //ts.setPadChar(' ');
    ts << return_period() << ' ' << pin_up_value << ' ' << pin_down_value << ' ' << beta;
    ts << endl;
}

void VoluntaryControl::cleanup()
{
    _osmer.forward(0);
    _file.close();
}
