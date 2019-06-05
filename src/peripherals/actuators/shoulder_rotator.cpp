#include "shoulder_rotator.h"

ShoulderRotator::ShoulderRotator()
    : Actuator("Shoulder Rotator")
{
    connect("/dev/ttyAMA0", B230400, 0x82, RoboClaw::M2);

    _menu.set_title(QString("Shoulder Rotator - ") + read_firmware_version());
    _menu.set_code(QString("shoulder"));

    read_params_limits(-45., 45.);
    read_params_technical(4600, 100);
    read_params_velocity(0.075, 0.006, 0, 667500);
    read_params_position(20., 0., 0., 0., 0., -180., 180.);
}

void ShoulderRotator::set_velocity_safe(double deg_s)
{
    set_velocity(deg_s);
}
