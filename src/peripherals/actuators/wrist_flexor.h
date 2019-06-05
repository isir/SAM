#ifndef WRISTFLEXOR_H
#define WRISTFLEXOR_H

#include "actuator.h"

class WristFlexor : public Actuator {
    Q_OBJECT
public:
    WristFlexor();

    void calibrate();
};

#endif // WRISTFLEXOR_H
