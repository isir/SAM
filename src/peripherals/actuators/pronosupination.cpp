#include "pronosupination.h"

PronoSupination::PronoSupination()
    : Actuator("Pronosupination")
{
    connect("/dev/ttyAMA0", B230400, 0x80, RoboClaw::M2);

    _menu->set_description(QString::fromStdString("Pronosupination - " + read_firmware_version()));
    _menu->set_code(QString("pronosup"));

    read_params_limits(-100., 0.);
    read_params_technical(23422, 100);
    read_params_velocity(4.2, 0.56, 0, 6000);
    read_params_position(51., 1.4, 428., 60., 0., -180., 180.);
}

void PronoSupination::set_velocity_safe(double deg_s)
{
    set_velocity(deg_s);
}
