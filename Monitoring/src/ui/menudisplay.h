#ifndef MENUDISPLAY_H
#define MENUDISPLAY_H

#include "../src/utils/mqttclient.h"
#include <QWidget>

namespace Ui {
class MenuDisplay;
}

class MenuDisplay : public QWidget {
    Q_OBJECT

public:
    explicit MenuDisplay(QWidget* parent = nullptr);
    ~MenuDisplay();

public slots:
    void setup();

private:
    Ui::MenuDisplay* ui;
    MqttClient& _mqtt;

private slots:
    void send_callback();
    void mqtt_message_callback(QMqttMessage msg);
};

#endif // MENUDISPLAY_H
