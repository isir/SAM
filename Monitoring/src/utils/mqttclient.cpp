#include "mqttclient.h"
#include <QDebug>

MqttClient::MqttClient()
    : QMqttClient()
{
    QObject::connect(this, &QMqttClient::stateChanged, this, &MqttClient::on_state_changed);
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

void MqttClient::connect_to_host(QString hostname, int port)
{
    setHostname(hostname);
    setPort(port);
    connectToHost();
}

void MqttClient::on_state_changed(QMqttClient::ClientState state)
{
    qDebug() << "mqtt state : " << state;
}
