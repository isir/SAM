#ifndef BASICCONTROLLER_H
#define BASICCONTROLLER_H

#include "ui/consolemenu.h"
#include <QMutex>
#include <QString>
#include <QThread>

class BasicController : public QThread {
    Q_OBJECT
public:
    BasicController(int period_s = 1);
    virtual ~BasicController();

    void set_period(double seconds);
    ConsoleMenu& menu() { return _menu; }

public slots:
    virtual void stop();

protected:
    virtual bool setup() = 0;
    virtual void loop(double dt, double time) = 0;
    virtual void cleanup() = 0;

    ConsoleMenu _menu;

private:
    void run();

    QMutex _mutex;
    bool _loop_condition;
    double _period_s;
};

#endif // BASICCONTROLLER_H
