#ifndef BASICCONTROLLER_H
#define BASICCONTROLLER_H

#include "ui/menu_user.h"
#include "utils/named_object.h"
#include <QMutex>
#include <QString>
#include <QThread>
#include <QTimer>
#include <memory>

class ThreadedLoop : public QThread, public NamedObject, public MenuUser {
    Q_OBJECT
public:
    ThreadedLoop(QString name, double period_s = 1);
    virtual ~ThreadedLoop();

    void set_period(double seconds);
    void set_preferred_cpu(int cpu);
    void set_prio(int prio);
    double period() { return _period_s; }
    void enable_watchdog(int timeout_ms);

public slots:
    virtual void stop();

protected:
    virtual bool setup() = 0;
    virtual void loop(double dt, double time) = 0;
    virtual void cleanup() = 0;

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
