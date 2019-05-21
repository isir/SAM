#include "osmer_elbow.h"

OsmerElbow::OsmerElbow(std::shared_ptr<QMqttClient> mqtt)
    : Actuator("Osmer", mqtt)
{
    connect("/dev/ttyAMA0", B230400, 0x80, RoboClaw::M1);

    _menu.set_title(QString("Custom Elbow - ") + read_firmware_version());
    _menu.set_code(QString("elbow"));

    read_params_limits(-100., 0.);
    read_params_technical(23422, 100);
    read_params_velocity(0.0387, 0.0029, 0, 1295000);
    read_params_position(68., 0., 0., 0., 0., -180., 180.);
}

void OsmerElbow::calibrate()
{
    Actuator::calibrate(15, 2, 0.5, false);
    move_to(0, 30, true);
}
