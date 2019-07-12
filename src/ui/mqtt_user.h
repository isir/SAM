#ifndef MQTT_USER_H
#define MQTT_USER_H

#include "utils/named_object.h"
#include <QMqttClient>

class MqttUser : public NamedObject {
public:
    typedef enum {
        AUTOCONNECT,
        NO_AUTOCONNECT
    } Autoconnection;

    virtual ~MqttUser();

    void connect_to_mqtt_server();

protected:
    MqttUser(QString client_id, Autoconnection autoconnect = AUTOCONNECT, NamedObject* parent = nullptr);

    QMqttClient _mqtt;
};

#endif // MQTT_USER_H
