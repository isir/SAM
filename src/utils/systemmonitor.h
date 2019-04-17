#ifndef SYSTEMMONITOR_H
#define SYSTEMMONITOR_H

#include <QFile>
#include <QMqttClient>
#include <QObject>
#include <QTimer>
#include <memory>

class SystemMonitor : public QObject {
    Q_OBJECT
public:
    explicit SystemMonitor(std::shared_ptr<QMqttClient> mqtt, QObject* parent = nullptr);
    void start();

private:
    QFile _stat_file;
    QFile _temp_file;
    QTimer _timer;
    std::shared_ptr<QMqttClient> _mqtt;

private slots:
    void timer_callback();
};

#endif // SYSTEMMONITOR_H
