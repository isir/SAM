#include "elbow_cybathlon.h"

ElbowCybathlon::ElbowCybathlon()
    : Actuator("Elbow Cybathlon")
{
    init("/dev/ttyAMA0", B230400, 0x83, RoboClaw::M1);

    _menu->set_description("Elbow Cybathlon - " + read_firmware_version());
    _menu->set_code("elbow");

    set_params_limits(0., 100.);
    set_params_technical(4065, 100);
    set_params_velocity(0.06070f, 0.00427f, 0, 898312);
    set_params_position(21.1098f, 0., 0., 0., 0., -180., 180.);
}

void ElbowCybathlon::calibrate()
{
    Actuator::calibrate(-5, 2, 0.5, false);
    move_to(0, 30, true);
}

