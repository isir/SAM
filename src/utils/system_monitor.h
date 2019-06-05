#ifndef SYSTEMMONITOR_H
#define SYSTEMMONITOR_H

#include <QFile>
#include <QObject>
#include <QTimer>
#include <memory>

class SystemMonitor : public QObject {
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
