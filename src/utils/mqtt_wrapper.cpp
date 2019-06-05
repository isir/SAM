#include "mqtt_wrapper.h"
#include <QMutexLocker>

QMutex MqttWrapper::_mutex;

QMqttClient& MqttWrapper::instance()
{
    static QMqttClient _mqtt;
    return _mqtt;
}

qint32 MqttWrapper::publish(const QMqttTopicName& topic, const QByteArray& message, quint8 qos, bool retain)
{
    QMutexLocker lock(&_mutex);
    return instance().publish(topic, message, qos, retain);
}

QMqttSubscription* MqttWrapper::subscribe(const QMqttTopicFilter& topic, quint8 qos)
{
    QMutexLocker lock(&_mutex);
    return instance().subscribe(topic, qos);
}

MqttWrapper::MqttWrapper()
{
}
