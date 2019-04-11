#include "systemmonitor.h"
#include <QDebug>

SystemMonitor::SystemMonitor(QObject* parent)
    : QObject(parent)
{
    _stat_file.setFileName("/proc/stat");
    _stat_file.open(QIODevice::ReadOnly);

    QObject::connect(&_timer, &QTimer::timeout, this, &SystemMonitor::timer_callback);
    _timer.setInterval(1000);
    _timer.start();
}

void SystemMonitor::timer_callback()
{
    static int old_usr[5] = { 0 }, old_nice[5] = { 0 }, old_sys[5] = { 0 }, old_idle[5] = { 0 };
    _stat_file.reset();
    double cpu_load[5];

    QByteArray msg;
    for (unsigned int i = 0; i < 5; ++i) {
        QStringList cpu_info = QString(_stat_file.readLine()).split(' ', QString::SkipEmptyParts);
        if (cpu_info.size() > 5) {
            int usr = cpu_info[1].toInt();
            int nice = cpu_info[2].toInt();
            int sys = cpu_info[3].toInt();
            int idle = cpu_info[4].toInt();
            cpu_load[i] = 1. - static_cast<double>(idle - old_idle[i]) / (usr + nice + sys + idle - old_usr[i] - old_nice[i] - old_sys[i] - old_idle[i]);
            old_usr[i] = usr;
            old_nice[i] = nice;
            old_sys[i] = sys;
            old_idle[i] = idle;
        }
        msg.append(QByteArray::number(cpu_load[i], 'f', 2) + " ");
    }
    MqttClient::instance().publish(QString("system/cpu_load"), msg);
}
