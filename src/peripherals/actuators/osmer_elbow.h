#ifndef OSMERELBOW_H
#define OSMERELBOW_H

#include "actuator.h"

class OsmerElbow : public Actuator {
    Q_OBJECT
public:
    OsmerElbow(std::shared_ptr<QMqttClient> mqtt);

    void calibrate();
};
#endif // OSMERELBOW_H
