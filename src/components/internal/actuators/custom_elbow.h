#ifndef CUSTOMELBOW_H
#define CUSTOMELBOW_H

#include "actuator.h"

class CustomElbow : public Actuator {
public:
    CustomElbow();

    void calibrate();
};

#endif // CUSTOMELBOW_H
