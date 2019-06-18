#include "custom_elbow.h"

CustomElbow::CustomElbow()
    : Actuator("Custom Elbow")
{
    connect("/dev/ttyAMA0", B230400, 0x82, RoboClaw::M1);

    _menu->set_description(QString("Custom Elbow - ") + read_firmware_version());
    _menu->set_code(QString("elbow"));

    read_params_limits(-100., 0.);
    read_params_technical(4065, 100);
    read_params_velocity(0.199, 0.014, 0, 268700);
    read_params_position(19.3, 0., 0., 0., 0., -180., 180.);
}

void CustomElbow::calibrate()
{
    Actuator::calibrate(15, 2, 0.5, false);
    move_to(0, 30, true);
}
