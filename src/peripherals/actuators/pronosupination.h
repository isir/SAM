#ifndef PRONOSUPINATION_H
#define PRONOSUPINATION_H

#include "actuator.h"

class PronoSupination : public Actuator {
    Q_OBJECT
public:
    PronoSupination(std::shared_ptr<QMqttClient> mqtt);

    void set_velocity_safe(double deg_s);
};

#endif // PRONOSUPINATION_H
