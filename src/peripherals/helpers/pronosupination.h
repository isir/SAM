#ifndef PRONOSUPINATION_H
#define PRONOSUPINATION_H

#include <QSettings>
#include "peripherals/roboclaw/client.h"
#include "ui/consolemenu.h"

class PronoSupination : public RoboClaw::Client
{
    Q_OBJECT
public:
    ~PronoSupination();

    static PronoSupination& instance();
    ConsoleMenu& menu();

private:
    PronoSupination();

    ConsoleMenu _menu;
    QSettings _settings;

private slots:
    void on_exit();
};

#endif // PRONOSUPINATION_H
