#include "mqtt_user.h"

MqttUser::MqttUser()
{
    if (!_mqtt.connect("127.0.0.1", 1883)) {
        throw std::runtime_error("Failed to connect to the MQTT broker");
    }
}

MqttUser::~MqttUser()
{
}
