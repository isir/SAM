#ifndef OSMERELBOW_H
#define OSMERELBOW_H

#include "actuator.h"

class OsmerElbow : public Actuator {
public:
    OsmerElbow();

    void calibrate();
};
#endif // OSMERELBOW_H
