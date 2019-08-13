#include "menu_broker.h"
#include "menu_backend.h"

MenuBroker::MenuBroker()
    : Worker("menu_broker")
    , _active_menu(nullptr)
{
}

MenuBroker::~MenuBroker()
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
    std::lock_guard<std::mutex> lock(_mutex);
    _inputs.push(input);
    do_work();
}

void MenuBroker::show_menu(std::string title, std::map<std::string, std::shared_ptr<MenuItem>> items)
{
    for (auto f : _frontends) {
        f->show_menu(title, items);
    }
}

void MenuBroker::work()
{
    std::unique_lock<std::mutex> lock(_mutex);
    if (!_inputs.empty()) {
        std::string input = _inputs.front();
        _inputs.pop();
        if (_active_menu) {
            lock.unlock();
            _active_menu->handle_input(input);
        }
    }
}
