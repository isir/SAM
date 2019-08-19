#ifndef PRONOSUPINATION_H
#define PRONOSUPINATION_H

#include "actuator.h"

class PronoSupination : public Actuator {
public:
    PronoSupination();

    void set_velocity_safe(double deg_s);
};

#endif // PRONOSUPINATION_H
