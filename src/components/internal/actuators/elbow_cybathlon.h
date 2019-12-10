#ifndef ELBOWCYBATHLON_H
#define ELBOWCYBATHLON_H

#include "actuator.h"

class ElbowCybathlon : public Actuator
{
public:
    ElbowCybathlon();
    void calibrate();
};

#endif // ELBOWCYBATHLON_H
