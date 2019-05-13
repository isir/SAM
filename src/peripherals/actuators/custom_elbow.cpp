#include "custom_elbow.h"
#include <QCoreApplication>
#include <QTime>
#include <iostream>

CustomElbow::CustomElbow(std::shared_ptr<QMqttClient> mqtt)
    : Actuator("Custom Elbow", mqtt)
{
    connect("/dev/ttyAMA0", 0x82, 230400, RoboClaw::Client::M1);
    read_params_limits(-100., 0.);
    read_params_technical(4065, 100);
    read_params_velocity(0.199, 0.014, 0, 268700);
    read_params_position(19.3, 0., 0., 0., 0., -180., 180.);

    _menu.set_title(QString("Custom Elbow - ") + read_firmware_version());
    _menu.set_code(QString("elbow"));
}

void CustomElbow::calibrate()
{
    Actuator::calibrate(15, 2, 0.5, false);
    move_to(0, 30);
}
