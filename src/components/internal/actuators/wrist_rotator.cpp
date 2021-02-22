#include "wrist_rotator.h"

WristRotator::WristRotator()
    : Actuator("Wrist Rotator")
{
    init("/dev/ttyAMA0", B230400, 0x81, RoboClaw::M1);

    _menu->set_description("Wrist Rotator - " + read_firmware_version());
    _menu->set_code("pronosup");

    set_params_limits(-60., 35.);
    set_params_technical(1000, 100);
    set_params_velocity(0.23f, 0.015f, 0, 221000);
    set_params_position(27., 0., 0., 0., 0., -180., 180.);
}

void WristRotator::calibrate()
{
    set_encoder_position(0);
    set_calibrated(true);
    //Actuator::calibrate(-15, -100, 0.5);
    //move_to(0, 40, true);
}
