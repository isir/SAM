#include "pronosupination.h"

PronoSupination::PronoSupination()
    : Actuator("Pronosupination")
{
    init("/dev/ttyAMA0", B230400, 0x80, RoboClaw::M2);

    _menu->set_description("Pronosupination - " + read_firmware_version());
    _menu->set_code("pronosup");

    set_params_limits(-100., 0.);
    set_params_technical(102, 100);
    set_params_velocity(4.15f, 0.4f, 0, 7500);
    set_params_position(10., 0., 0., 0., 0., -180., 180.);
}

void PronoSupination::set_velocity_safe(double deg_s)
{
    set_velocity(deg_s);
}

void PronoSupination::calibrate()
{
    Actuator::set_encoder_position(0);
    Actuator::set_calibrated(true);
}
