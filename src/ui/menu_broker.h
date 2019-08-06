#ifndef MENU_BROKER_H
#define MENU_BROKER_H

#include "menu_frontend.h"
#include "menu_item.h"
#include <map>
#include <memory>
#include <mutex>

class MenuBackend;

class MenuBroker {
public:
    explicit MenuBroker();

    void register_frontend(MenuFrontend* f);
    void set_active_menu(MenuBackend* menu);
    void handle_input(std::string input);
    void show_menu(std::string title, std::map<std::string, std::shared_ptr<MenuItem>> items);

private:
    MenuBackend* _active_menu;
    std::vector<MenuFrontend*> _frontends;
    std::mutex _mutex;
};

#endif // MENU_BROKER_H
