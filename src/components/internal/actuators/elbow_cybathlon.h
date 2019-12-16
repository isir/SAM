#ifndef ELBOWCYBATHLON_H
#define ELBOWCYBATHLON_H

#include "actuator.h"

class ElbowCybathlon : public Actuator
{
public:
    ElbowCybathlon();
    void calibrate();
    void move_to(double deg, double speed, bool block = false);
    void set_max_velocity(double speed);

private:
    double _max_velocity;

};

#endif // ELBOWCYBATHLON_H
