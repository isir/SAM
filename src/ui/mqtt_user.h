#ifndef MQTT_USER_H
#define MQTT_USER_H

#include "mosquitto/client.h"
#include "utils/named_object.h"

class MqttUser {
public:
    virtual ~MqttUser() = 0;

protected:
    MqttUser();

    Mosquitto::Client _mqtt;
};

#endif // MQTT_USER_H
