#ifndef WRISTROTATOR_H
#define WRISTROTATOR_H

#include "actuator.h"

class WristRotator : public Actuator {

public:
    WristRotator();

    void calibrate();
};

#endif // WRISTROTATOR_H
