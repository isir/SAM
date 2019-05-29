#ifndef MQTTCONNECT_H
#define MQTTCONNECT_H

#include "utils/mqttclient.h"
#include <QWidget>

namespace Ui {
class MqttConnect;
}

class MqttConnect : public QWidget {
    Q_OBJECT

public:
    explicit MqttConnect(QWidget* parent = nullptr);
    ~MqttConnect();

private:
    Ui::MqttConnect* ui;

private slots:
    void button_callback();
    void mqtt_state_callback(QMqttClient::ClientState state);
};

#endif // MQTTCONNECT_H
