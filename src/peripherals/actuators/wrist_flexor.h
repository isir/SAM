#ifndef WRISTFLEXOR_H
#define WRISTFLEXOR_H

#include "actuator.h"

class WristFlexor : public Actuator {
public:
    WristFlexor();

    void calibrate();
};

#endif // WRISTFLEXOR_H
