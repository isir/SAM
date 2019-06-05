#ifndef BASICCONTROLLER_H
#define BASICCONTROLLER_H

#include "ui/console_menu.h"
#include <QMutex>
#include <QString>
#include <QThread>
#include <QTimer>
#include <memory>

class BasicController : public QThread {
    Q_OBJECT
public:
    BasicController(double period_s = 1);
    virtual ~BasicController();

    void set_period(double seconds);
    void set_preferred_cpu(int cpu);
    void set_prio(int prio);
    double period() { return _period_s; }
    ConsoleMenu& menu() { return _menu; }
    void enable_watchdog(int timeout_ms);

public slots:
    virtual void stop();

protected:
    virtual bool setup() = 0;
    virtual void loop(double dt, double time) = 0;
    virtual void cleanup() = 0;

    ConsoleMenu _menu;
    bool _loop_condition;
    QMutex _mutex;

protected slots:
    virtual void unresponsive_callback();

private:
    void run();

    double _period_s;
    int _pref_cpu;
    int _prio;
    QTimer _watchdog_timer;

signals:
    void ping();
};

#endif // BASICCONTROLLER_H
