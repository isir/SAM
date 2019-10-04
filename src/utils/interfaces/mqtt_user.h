#ifndef MQTT_USER_H
#define MQTT_USER_H

#include "ux/mosquittopp/client.h"

class MqttUser {
public:
    virtual ~MqttUser() = 0;

protected:
    MqttUser();

    Mosquittopp::Client _mqtt;
};

#endif // MQTT_USER_H
