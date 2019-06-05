#ifndef MQTT_WRAPPER_H
#define MQTT_WRAPPER_H

#include <QMqttClient>
#include <QMutex>

class MqttWrapper {
public:
    static QMqttClient& instance();

    static qint32 publish(const QMqttTopicName& topic, const QByteArray& message = QByteArray(), quint8 qos = 0, bool retain = false);
    static QMqttSubscription* subscribe(const QMqttTopicFilter& topic, quint8 qos = 0);

    MqttWrapper(const MqttWrapper&) = delete;
    void operator=(const MqttWrapper&) = delete;

private:
    MqttWrapper();
    static QMutex _mutex;
};

#define mqtt MqttWrapper::instance
#define mqtt_pub MqttWrapper::publish
#define mqtt_sub MqttWrapper::subscribe

#endif // MQTT_WRAPPER_H
