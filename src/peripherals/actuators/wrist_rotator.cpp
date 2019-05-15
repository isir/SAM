#include "wrist_rotator.h"
#include <QCoreApplication>
#include <QTime>
#include <iostream>

WristRotator::WristRotator(std::shared_ptr<QMqttClient> mqtt)
    : Actuator("Wrist Rotator", mqtt)
{
    Actuator::connect("/dev/ttyAMA0", 0x81, 230400, RoboClaw::RoboClaw::M1);
    read_params_limits(-35., 35.);
    read_params_technical(1000, 100);
    read_params_velocity(0.23, 0.015, 0, 221000);
    read_params_position(27., 0., 0., 0., 0., -180., 180.);

    _menu.set_title(QString("Wrist Rotator - ") + read_firmware_version());
    _menu.set_code(QString("pronosup"));
}

void WristRotator::calibrate()
{
    Actuator::calibrate(-15, -100, 0.5);
    move_to(0, 40);
}
