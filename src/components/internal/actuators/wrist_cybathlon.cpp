#include "wrist_cybathlon.h"

WristCybathlon::WristCybathlon()
    : Actuator("Wrist Cybathlon")
{
    init("/dev/ttyAMA0", B230400, 0x83, RoboClaw::M2);

    _menu->set_description("Wrist Rotator Cybathlon - " + read_firmware_version());
    _menu->set_code("pronosup");

    set_params_limits(-60., 35.);
    set_params_technical(1000, 100);
    set_params_velocity(5.63265f, 0.58411f, 0, 5625);
    set_params_position(11.5974f, 0., 0., 0., 0., -180., 180.);
}

void WristCybathlon::set_velocity_safe(double deg_s)
{
    set_velocity(deg_s);
}
