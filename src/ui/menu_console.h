#ifndef MENU_CONSOLE_H
#define MENU_CONSOLE_H

#include "console_input.h"
#include "menu_frontend.h"
#include <QThread>

class MenuConsole : public MenuFrontend {
    Q_OBJECT
public:
    MenuConsole();
    ~MenuConsole();

public slots:
    void show_menu_callback(QString title, QMap<QString, std::shared_ptr<MenuItem>> items) override;

private:
    ConsoleInput _input;
    QThread _thread;

signals:
    void ready_to_read();
};

#endif // MENU_CONSOLE_H
