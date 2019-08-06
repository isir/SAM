#ifndef MENU_CONSOLE_H
#define MENU_CONSOLE_H

#include "menu_frontend.h"
#include <map>
#include <string>
#include <thread>

class MenuConsole : public MenuFrontend {
public:
    MenuConsole();
    ~MenuConsole() override;

    void show_menu(std::string title, std::map<std::string, std::shared_ptr<MenuItem>> items) override;
    void show_message(std::string msg) override;
    void read_line();

private:
    std::thread _thread;
};

#endif // MENU_CONSOLE_H
