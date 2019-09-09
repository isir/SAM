#include "osmer_elbow.h"

OsmerElbow::OsmerElbow()
    : Actuator("Osmer")
{
    init("/dev/ttyAMA0", B230400, 0x80, RoboClaw::M1);

    _menu->set_description("Custom Elbow - " + read_firmware_version());
    _menu->set_code("elbow");

    set_params_limits(-100., 0.);
    set_params_technical(23422, 100);
    set_params_velocity(0.0387f, 0.0029f, 0, 1295000);
    set_params_position(68., 0., 0., 0., 0., -180., 180.);
}

void OsmerElbow::calibrate()
{
    Actuator::calibrate(15, 2, 0.5, false);
    move_to(0, 30, true);
}
