#include "wrist_flexor.h"

WristFlexor::WristFlexor()
    : Actuator("Wrist Flexor")
{
    init("/dev/ttyAMA0", B230400, 0x81, RoboClaw::M2);

    _menu->set_description("Wrist Flexor - " + read_firmware_version());
    _menu->set_code("flex");

    set_params_limits(-35., 20.);
    set_params_technical(12000, 100);
    set_params_velocity(0.177f, 0.012f, 0, 335500);
    set_params_position(20., 0., 0., 0., 0., -180., 180.);
}

void WristFlexor::calibrate()
{
    Actuator::calibrate(-15, -45, 0.5);
    move_to(0, 20, true);
}
