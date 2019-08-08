#include "mqtt_user.h"

MqttUser::MqttUser()
{
    _mqtt.connect("127.0.0.1", 1883);
}

MqttUser::~MqttUser()
{
}
