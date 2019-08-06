#ifndef MQTT_USER_H
#define MQTT_USER_H

#include "mosquittopp/client.h"
#include "utils/named_object.h"

class MqttUser {
public:
    virtual ~MqttUser() = 0;

protected:
    MqttUser();

    Mosquittopp::Client _mqtt;
};

#endif // MQTT_USER_H
