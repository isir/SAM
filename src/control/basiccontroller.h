#ifndef BASICCONTROLLER_H
#define BASICCONTROLLER_H

#include <QThread>
#include <QMutex>
#include <QString>
#include "ui/consolemenu.h"

class BasicController : public QThread
{
    Q_OBJECT
public:
    BasicController(int period_s = 1);
    virtual ~BasicController();

    void set_period(double seconds);
    double return_period(){return _period_s;}
    ConsoleMenu& menu() { return _menu; }

public slots:
    virtual void stop();

protected:
    virtual void setup() = 0;
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
