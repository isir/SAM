#include "custom_elbow.h"

CustomElbow::CustomElbow()
    : Actuator("Custom Elbow")
{
    init("/dev/ttyAMA0", B230400, 0x82, RoboClaw::M1);

    _menu->set_description("Custom Elbow - " + read_firmware_version());
    _menu->set_code("elbow");

    set_params_limits(-115., 0.);
    set_params_technical(4065, 100);
    set_params_velocity(0.199f, 0.014f, 0, 268700);
    set_params_position(19.3f, 0., 0., 0., 0., -180., 180.);
}

void CustomElbow::calibrate()
{
    Actuator::calibrate(15, 2, 0.5, false);
    move_to(0, 30, true);
}
