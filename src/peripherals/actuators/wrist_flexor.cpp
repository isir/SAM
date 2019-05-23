#include "wrist_flexor.h"
#include <QCoreApplication>
#include <QDebug>
#include <QTime>
#include <iostream>

WristFlexor::WristFlexor(std::shared_ptr<QMqttClient> mqtt)
    : Actuator("Wrist Flexor", mqtt)
{
    connect("/dev/ttyAMA0", B230400, 0x81, RoboClaw::M2);

    _menu.set_title(QString("Wrist Flexor - ") + read_firmware_version());
    _menu.set_code(QString("flex"));

    read_params_limits(-35., 20.);
    read_params_technical(12000, 100);
    read_params_velocity(0.177, 0.012, 0, 335500);
    read_params_position(20., 0., 0., 0., 0., -180., 180.);
}

void WristFlexor::calibrate()
{
    Actuator::calibrate(-15, -45, 0.5);
    move_to(0, 20, true);
}
