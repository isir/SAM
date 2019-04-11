#include "mqttclient.h"
#include <QDebug>

MqttClient::MqttClient()
    : QMqttClient()
{
    QObject::connect(this, &QMqttClient::stateChanged, this, &MqttClient::on_state_changed);
    setHostname("192.168.0.130");
    setPort(1883);
    connectToHost();
}

MqttClient::~MqttClient()
{
    disconnectFromHost();
}

MqttClient& MqttClient::instance()
{
    static MqttClient client;
    return client;
}

void MqttClient::on_state_changed(QMqttClient::ClientState state)
{
    qDebug() << state;
}
