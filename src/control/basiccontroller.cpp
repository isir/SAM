#include "basiccontroller.h"
#include <QDebug>
#include <QMutexLocker>
#include <QTime>

BasicController::BasicController(double period_s)
    : _period_s(period_s)
{
    QObject::connect(&_menu, &ConsoleMenu::finished, this, &BasicController::stop);
    _menu.addItem(ConsoleMenuItem("Start loop", "start", [this](QString) { this->start(); }));
    _menu.addItem(ConsoleMenuItem("Stop loop", "stop", [this](QString) { this->stop(); }));
}

BasicController::~BasicController()
{
    stop();
}

void BasicController::set_period(double seconds)
{
    QMutexLocker locker(&_mutex);
    _period_s = seconds;
}

void BasicController::stop()
{
    _mutex.lock();
    _loop_condition = false;
    _mutex.unlock();
    wait();
}

void BasicController::run()
{
    QTime t;
    double old_time = 0, current_time = 0;

    double dt = 0;
    unsigned int cnt = 0;
    double min = std::numeric_limits<double>::max(), max = 0, avg = 0;

    if (!setup()) {
        return;
    }

    t.start();
    _loop_condition = true;

    while (true) {
        while (true) {
            current_time = t.elapsed() / 1000.;
            dt = current_time - old_time;

            QMutexLocker locker(&_mutex);
            if (dt >= _period_s)
                break;
        }
        old_time = current_time;

        loop(dt, current_time);

        ++cnt;
        if (dt < min)
            min = dt;
        else if (dt > max)
            max = dt;
        avg = (avg * (cnt - 1) + dt) / cnt;

        QMutexLocker locker(&_mutex);
        if (!_loop_condition)
            break;
    }

    cleanup();
    qInfo() << "Thread finished - Avg loop time = " << avg * 1000. << "ms, min = " << min * 1000. << "ms, max = " << max * 1000. << "ms";
}
