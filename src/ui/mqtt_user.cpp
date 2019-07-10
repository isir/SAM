#include "mqtt_user.h"
#include "utils/settings.h"

MqttUser::MqttUser(QString client_id, Autoconnection autoconnect)
    : NamedObject(client_id)
{
    _mqtt.setClientId(client_id);
    if (autoconnect == AUTOCONNECT) {
        connect_to_mqtt_server();
    }
}

MqttUser::~MqttUser()
{
}

void MqttUser::connect_to_mqtt_server()
{
    Settings s;

    s.beginGroup("MQTT");
    _mqtt.setHostname(s.value("hostname", "127.0.0.1").toString());
    _mqtt.setPort(static_cast<quint16>(s.value("port", 1883).toUInt()));
    _mqtt.connectToHost();
    s.endGroup();
}
