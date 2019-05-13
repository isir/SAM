#ifndef PRONOSUPINATION_H
#define PRONOSUPINATION_H

#include "peripherals/roboclaw/client.h"
#include "ui/consolemenu.h"
#include "utils/settings.h"
#include <QMqttClient>
#include <memory>

class PronoSupination : public RoboClaw::Client {
    Q_OBJECT
public:
    PronoSupination(std::shared_ptr<QMqttClient> mqtt);
    ~PronoSupination();

    ConsoleMenu& menu();

private:
    ConsoleMenu _menu;
    Settings _settings;

private slots:
    void on_exit();
};

#endif // PRONOSUPINATION_H
