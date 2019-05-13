#include "pronosupination.h"
#include "peripherals/roboclaw/factory.h"
#include "peripherals/roboclaw/server.h"
#include <iostream>

PronoSupination::PronoSupination(std::shared_ptr<QMqttClient> mqtt)
    : Actuator("Pronosupination", mqtt)
{
    connect("/dev/ttyAMA0", 0x80, 230400, RoboClaw::Client::M2);
    read_params_limits(-100., 0.);
    read_params_technical(23422, 100);
    read_params_velocity(4.2, 0.56, 0, 6000);
    read_params_position(51., 1.4, 428., 60., 0., -180., 180.);

    _menu.set_title(QString("Pronosupination - ") + read_firmware_version());
    _menu.set_code(QString("pronosup"));
}
