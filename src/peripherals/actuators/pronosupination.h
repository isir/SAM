#ifndef PRONOSUPINATION_H
#define PRONOSUPINATION_H

#include "actuator.h"

class PronoSupination : public Actuator {
    Q_OBJECT
public:
    PronoSupination(std::shared_ptr<QMqttClient> mqtt);
};

#endif // PRONOSUPINATION_H
