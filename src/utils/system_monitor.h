#ifndef SYSTEMMONITOR_H
#define SYSTEMMONITOR_H

#include "ui/mqtt_user.h"
#include <QFile>
#include <QObject>
#include <QTimer>
#include <memory>

class SystemMonitor : public QObject, public MqttUser {
    Q_OBJECT
public:
    explicit SystemMonitor(QObject* parent = nullptr);
    void start();

private:
    QFile _stat_file;
    QFile _temp_file;
    QTimer _timer;

private slots:
    void timer_callback();
};

#endif // SYSTEMMONITOR_H
