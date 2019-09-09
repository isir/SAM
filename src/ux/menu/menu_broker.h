#ifndef MENU_BROKER_H
#define MENU_BROKER_H

#include "menu_item.h"
#include "ui/menu/menu_frontend.h"
#include "utils/worker.h"
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <queue>

class MenuBackend;

class MenuBroker : public Worker {
public:
    explicit MenuBroker();
    ~MenuBroker() override;

    void register_frontend(MenuFrontend* f);
    void set_active_menu(MenuBackend* menu);
    void handle_input(std::string input);
    void show_menu(std::string title, std::map<std::string, std::shared_ptr<MenuItem>> items);

private:
    void work() override;

    MenuBackend* _active_menu;
    std::vector<MenuFrontend*> _frontends;

    std::queue<std::string> _inputs;
    std::mutex _mutex;
};

#endif // MENU_BROKER_H
