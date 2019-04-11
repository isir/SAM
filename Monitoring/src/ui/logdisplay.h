#ifndef LOGDISPLAY_H
#define LOGDISPLAY_H

#include "../src/utils/mqttclient.h"
#include <QList>
#include <QWidget>

namespace Ui {
class LogDisplay;
}

class LogDisplay : public QWidget {
    Q_OBJECT

public:
    explicit LogDisplay(QWidget* parent = nullptr);
    ~LogDisplay();

public slots:
    void setup();

private:
    void display_message(QMqttMessage msg);

    Ui::LogDisplay* ui;
    MqttClient& _mqtt;
    QList<QMqttMessage> _messages;

private slots:
    void refresh();
    void mqtt_message_callback(QMqttMessage msg);
};

#endif // LOGDISPLAY_H
