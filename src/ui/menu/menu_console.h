#ifndef MENU_CONSOLE_H
#define MENU_CONSOLE_H

#include "menu_frontend.h"
#include "utils/worker.h"

class MenuConsole : public MenuFrontend, public Worker {
public:
    MenuConsole();
    ~MenuConsole() override;

    void show_menu(std::string title, std::map<std::string, std::shared_ptr<MenuItem>> items) override;
    void show_message(std::string msg) override;

private:
    void work() override;
};

#endif // MENU_CONSOLE_H
