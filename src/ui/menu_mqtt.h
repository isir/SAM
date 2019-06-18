#ifndef MENU_MQTT_H
#define MENU_MQTT_H

#include "menu_frontend.h"
#include "mqtt_user.h"
#include <QByteArray>

class MenuMQTT : public MenuFrontend, public MqttUser {
    Q_OBJECT
public:
    MenuMQTT();

public slots:
    void show_message(QByteArray msg);
    void show_menu_callback(QString title, QMap<QString, std::shared_ptr<MenuItem>> items) override;

private slots:
    void mqtt_connected_callback();
    void mqtt_message_received_callback(const QMqttMessage msg);

private:
    QByteArray _unpublished_msg;
};

#endif // MENU_MQTT_H
