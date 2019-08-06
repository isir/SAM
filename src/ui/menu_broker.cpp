#include "menu_broker.h"
#include "menu_backend.h"

MenuBroker::MenuBroker()
    : _active_menu(nullptr)
{
}

void MenuBroker::register_frontend(MenuFrontend* f)
{
    if (f) {
        _frontends.push_back(f);
    }
}

void MenuBroker::set_active_menu(MenuBackend* menu)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _active_menu = menu;
}

void MenuBroker::handle_input(std::string input)
{
    if (_active_menu) {
        _active_menu->handle_input(input);
    }
}

void MenuBroker::show_menu(std::string title, std::map<std::string, std::shared_ptr<MenuItem>> items)
{
    for (auto f : _frontends) {
        f->show_menu(title, items);
    }
}
