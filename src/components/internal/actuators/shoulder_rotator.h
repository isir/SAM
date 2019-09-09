#ifndef SHOULDERROTATOR_H
#define SHOULDERROTATOR_H

#include "actuator.h"

class ShoulderRotator : public Actuator {

public:
    ShoulderRotator();

    void set_velocity_safe(double deg_s);
};

#endif // SHOULDERROTATOR_H
