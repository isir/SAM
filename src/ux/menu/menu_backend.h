#ifndef MENU_BACKEND_H
#define MENU_BACKEND_H

#include "menu_broker.h"
#include "menu_item.h"
#include <functional>
#include <map>
#include <memory>
#include <string>

class MenuBackend : public MenuItem {
public:
    MenuBackend(
        std::string code = std::string(), std::string description = std::string(), std::function<void(void)> exit_callback = [] {});
    ~MenuBackend();

    void add_exit(std::function<void(std::string)> callback);
    void add_item(std::string code, std::string description, std::function<void(std::string)> callback);
    void add_item(std::shared_ptr<MenuItem> item);

    template <typename T>
    void add_submenu_from_user(const std::unique_ptr<T>& obj)
    {
        if (obj) {
            add_item(obj->menu());
        }
    }

    static MenuBroker broker;
    void handle_input(std::string input);

    void set_activated_callback(std::function<void(void)> f);
    void activate();
    void activate_item(std::string code, std::string args = std::string());

protected:
    virtual void on_exit();

private:
    void set_parent(MenuBackend* parent);

    MenuBackend* _parent;

    std::function<void(void)> _activated_callback;
    std::function<void(void)> _exit_callback;

    std::map<std::string, std::shared_ptr<MenuItem>> _items;
};

#endif // MENU_BACKEND_H
