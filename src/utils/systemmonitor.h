#ifndef SYSTEMMONITOR_H
#define SYSTEMMONITOR_H

#include "mqttclient.h"
#include <QFile>
#include <QObject>
#include <QTimer>

class SystemMonitor : public QObject {
    Q_OBJECT
public:
    explicit SystemMonitor(QObject* parent = nullptr);

private:
    QFile _stat_file;
    QTimer _timer;

private slots:
    void timer_callback();
};

#endif // SYSTEMMONITOR_H
