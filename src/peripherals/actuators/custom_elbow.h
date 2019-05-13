#ifndef CUSTOMELBOW_H
#define CUSTOMELBOW_H

#include "actuator.h"

class CustomElbow : public Actuator {
    Q_OBJECT
public:
    CustomElbow(std::shared_ptr<QMqttClient> mqtt);

    void calibrate();
};

#endif // CUSTOMELBOW_H
