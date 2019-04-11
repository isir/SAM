#ifndef MQTTCLIENT_H
#define MQTTCLIENT_H

#include <QMqttClient>
#include <QObject>

class MqttClient : public QMqttClient {
    Q_OBJECT
public:
    static MqttClient& instance();
    ~MqttClient();

private:
    MqttClient();

private slots:
    void on_state_changed(QMqttClient::ClientState state);
};

#endif // MQTTCLIENT_H
