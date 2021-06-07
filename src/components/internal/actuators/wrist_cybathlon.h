#ifndef WRISTCYBATHLON_H
#define WRISTCYBATHLON_H

#include "actuator.h"

class WristCybathlon : public Actuator {
public:
    WristCybathlon();
    void set_velocity_safe(double deg_s);
    void calibrate();
};

#endif // WRIST_CYBATHLON_H
