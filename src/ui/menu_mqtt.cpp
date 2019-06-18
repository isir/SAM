#include "menu_mqtt.h"

MenuMQTT::MenuMQTT()
    : MenuFrontend()
    , MqttUser("menu_mqtt")
{
    connect_to_backend();
    QObject::connect(&_mqtt, &QMqttClient::connected, this, &MenuMQTT::mqtt_connected_callback);
}

void MenuMQTT::show_message(QByteArray msg)
{
    _mqtt.publish(QString("sam/menu/output"), msg, 0, true);
}

void MenuMQTT::show_menu_callback(QString title, QMap<QString, std::shared_ptr<MenuItem>> items)
{
    QByteArray buffer;
    QByteArray filler;
    if (!title.isEmpty()) {
        filler.fill('-', title.length() + 1);
        buffer.append(title + ":\r\n");
        buffer.append(filler + "\r\n");
    }
    foreach (std::shared_ptr<MenuItem> item, items) {
        buffer.append("[" + item->code() + "]" + " " + item->description() + "\r\n");
    }
    if (_mqtt.state() == QMqttClient::Connected) {
        _mqtt.publish(QString("sam/menu/output"), buffer, 0, true);
    } else {
        _unpublished_msg = buffer;
    }
}

void MenuMQTT::mqtt_connected_callback()
{
    _mqtt.setWillTopic("sam/menu/output");
    _mqtt.setWillRetain(true);
    _mqtt.setWillMessage("The embedded program crashed.");

    QMqttSubscription* sub = _mqtt.subscribe(QString("sam/menu/input"));
    QObject::connect(sub, &QMqttSubscription::messageReceived, this, &MenuMQTT::mqtt_message_received_callback);

    if (_unpublished_msg.size() > 0) {
        _mqtt.publish(QString("sam/menu/output"), _unpublished_msg, 0, true);
    }
}

void MenuMQTT::mqtt_message_received_callback(const QMqttMessage msg)
{
    emit input_received(QString::fromLatin1(msg.payload()));
}
