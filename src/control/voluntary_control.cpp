#include "voluntary_control.h"
#include "peripherals/roboclaw/factory.h"
#include "utils/check_ptr.h"

#include "qmath.h"
#include <QNetworkDatagram>
#include <iostream>
#include <wiringPi.h>

VoluntaryControl::VoluntaryControl(std::shared_ptr<SAM::Components> robot)
    : BasicController()
    , _robot(robot)
{
    if (!check_ptr(_robot->joints.elbow, _robot->joints.wrist_pronosup)) {
        throw std::runtime_error("Volontary Control is missing components");
    }

    _settings.beginGroup("VoluntaryControl");
    _pin_up = _settings.value("pin_up", 24).toInt();
    _pin_down = _settings.value("pin_down", 22).toInt();
    set_period(_settings.value("period", 0.01).toDouble());

    _menu->set_description("Voluntary Control");
    _menu->set_code("vc");
    _menu->add_item(_robot->joints.elbow->menu());

    pullUpDnControl(_pin_up, PUD_UP);
    pullUpDnControl(_pin_down, PUD_UP);
}

VoluntaryControl::~VoluntaryControl()
{
    _robot->joints.elbow->forward(0);
    _robot->joints.wrist_pronosup->forward(0);
    stop();
}

bool VoluntaryControl::setup()
{
    _robot->joints.wrist_pronosup->set_encoder_position(0);
    QString filename = QString("voluntary");

    int cnt = 0;
    QString extension = QString(".txt");
    do {
        ++cnt;
        QString suffix = QString("_") + QString::number(cnt);
        _file.setFileName(filename + suffix + extension);
    } while (_file.exists());

    if (!_file.open(QIODevice::ReadWrite)) {
        qCritical() << "Failed to open" << _file.fileName() << "-" << _file.errorString();
        return false;
    }
    _need_to_write_header = true;
    return true;
}

void VoluntaryControl::loop(double, double)
{
    static int prev_pin_up_value = 1, prev_pin_down_value = 1;
    int pin_down_value = digitalRead(_pin_down);
    int pin_up_value = digitalRead(_pin_up);

    /// ELBOW
    //    double beta = _osmer.angle() * M_PI / 180.;

    //    if (pin_down_value == 0 && prev_pin_down_value == 1) {
    //        _osmer.set_velocity(30);
    //    } else if (pin_up_value == 0 && prev_pin_up_value == 1) {
    //        _osmer.set_velocity(-30);
    //    } else if ((pin_down_value == 1 && pin_up_value == 1) && (prev_pin_down_value == 0 || prev_pin_up_value == 0)) {
    //        _osmer.set_velocity(0);
    //        //        sleep(2);
    //        //        _osmer.set_velocity(0);
    //    }

    /// WRIST
    double wristAngle = _robot->joints.wrist_pronosup->read_encoder_position();

    if (pin_down_value == 0 && prev_pin_down_value == 1) {
        _robot->joints.wrist_pronosup->move_to(6000, 5000, 6000, 35000);
    } else if (pin_up_value == 0 && prev_pin_up_value == 1) {
        _robot->joints.wrist_pronosup->move_to(6000, 5000, 6000, -35000);
    } else if ((pin_down_value == 1 && pin_up_value == 1) && (prev_pin_down_value == 0 || prev_pin_up_value == 0)) {
        _robot->joints.wrist_pronosup->forward(0);
    }

    prev_pin_down_value = pin_down_value;
    prev_pin_up_value = pin_up_value;

    optitrack_data_t data = _robot->sensors.optitrack->get_last_data();
    if (_need_to_write_header) {
        //        _file.write("period, btnUp, btnDown, beta");
        _file.write("period, btnUp, btnDown, wristAngle");
        for (int i = 0; i < data.nRigidBodies; i++) {
            _file.write(", ID, bTrackingValid, fError, qw, qx, qy, qz, x, y, z");
        }
        _file.write("\r\n");
        _need_to_write_header = false;
    }

    QTextStream ts(&_file);
    //ts.setPadChar(' ');
    //    ts << return_period() << ' ' << pin_up_value << ' ' << pin_down_value << ' ' << beta;
    ts << period() << ' ' << pin_up_value << ' ' << pin_down_value << ' ' << wristAngle;
    for (int i = 0; i < data.nRigidBodies; i++) {
        ts << ' ' << data.rigidBodies[i].ID << ' ' << data.rigidBodies[i].bTrackingValid << ' ' << data.rigidBodies[i].fError;
        ts << ' ' << data.rigidBodies[i].qw << ' ' << data.rigidBodies[i].qx << ' ' << data.rigidBodies[i].qy << ' ' << data.rigidBodies[i].qz;
        ts << ' ' << data.rigidBodies[i].x << ' ' << data.rigidBodies[i].y << ' ' << data.rigidBodies[i].z;
    }
    ts << endl;
}

void VoluntaryControl::cleanup()
{
    //_robot.elbow->forward(0);
    _robot->joints.wrist_pronosup->forward(0);
    _file.close();
}
