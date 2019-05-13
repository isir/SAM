#ifndef WRISTROTATOR_H
#define WRISTROTATOR_H

#include "actuator.h"

class WristRotator : public Actuator {

public:
    WristRotator(std::shared_ptr<QMqttClient> mqtt);

    void calibrate();
};

#endif // WRISTROTATOR_H
