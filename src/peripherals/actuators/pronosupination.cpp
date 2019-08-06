#include "pronosupination.h"

PronoSupination::PronoSupination()
    : Actuator("Pronosupination")
{
    init("/dev/ttyAMA0", B230400, 0x80, RoboClaw::M2);

    _menu->set_description("Pronosupination - " + read_firmware_version());
    _menu->set_code("pronosup");

    set_params_limits(-100., 0.);
    set_params_technical(23422, 100);
    set_params_velocity(4.2f, 0.56f, 0, 6000);
    set_params_position(51., 1.4f, 428., 60., 0., -180., 180.);
}

void PronoSupination::set_velocity_safe(double deg_s)
{
    set_velocity(deg_s);
}
