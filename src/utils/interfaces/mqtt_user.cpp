#include "mqtt_user.h"

MqttUser::MqttUser()
{
    if (!_mqtt.connect(MOSQUITTO_SERVER_IP, MOSQUITTO_SERVER_PORT)) {
        throw std::runtime_error("Failed to connect to the MQTT broker");
    }
}

MqttUser::~MqttUser()
{
}
