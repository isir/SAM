#include "elbow_cybathlon.h"

ElbowCybathlon::ElbowCybathlon()
    : Actuator("Elbow Cybathlon")
{
    init("/dev/ttyAMA0", B230400, 0x83, RoboClaw::M1);

    _menu->set_description("Elbow Cybathlon - " + read_firmware_version());
    _menu->set_code("elbow");

    _max_velocity = 30;

    set_params_limits(0, 100.);
    set_params_technical(4065, 100);
    set_params_velocity(0.10774f, 0.00718f, 0, 424875);
    set_params_position(16.67f, 0., 0., 0., 0., -180., 180.);
    //set_params_velocity(0.04060f, 0.00325f, 0, 897000);
    //set_params_position(11.90776f, 0., 0., 0., 0., -180., 180.);
}

void ElbowCybathlon::calibrate()
{
    Actuator::calibrate(-10, -15, 0.5, false);
    move_to(0, 15, true);
}

void ElbowCybathlon::move_to(double deg, double speed, bool block)
{
    if (speed > _max_velocity)
    {
        speed = _max_velocity;
    } else if (speed < -_max_velocity)
    {
        speed = -_max_velocity;
    }

    Actuator::move_to(deg,speed,block);

}

void ElbowCybathlon::set_max_velocity(double deg_s)
{
    _max_velocity = deg_s;
}
