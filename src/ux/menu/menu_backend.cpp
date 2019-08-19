#include "menu_backend.h"

MenuBroker MenuBackend::broker;

MenuBackend::MenuBackend(std::string code, std::string description, std::function<void(void)> exit_callback)
    : MenuItem(
        code, description, [this](std::string) { activate(); }, SUBMENU)
    , _parent(nullptr)
    , _activated_callback([] {})
    , _exit_callback(exit_callback)
{
    add_exit([this](std::string) { _exit_callback(); on_exit(); });
}

MenuBackend::~MenuBackend()
{
}

void MenuBackend::add_exit(std::function<void(std::string)> callback)
{
    add_item(std::make_shared<ExitItem>(callback));
}

void MenuBackend::add_item(std::string code, std::string description, std::function<void(std::string)> callback)
{
    add_item(std::make_shared<StandardItem>(code, description, callback));
}

void MenuBackend::add_item(std::shared_ptr<MenuItem> item)
{
    if (item) {
        _items[item->code()] = item;
    }
}

void MenuBackend::handle_input(std::string input)
{
    std::string key, input_key, args;
    auto i = _items.begin();
    for (; i != _items.end(); ++i) {
        key = (*i).first.substr(0, (*i).first.find_first_of(' '));
        input_key = input.substr(0, input.find_first_of(' '));
        if (key == input) {
            break;
        } else if (key == input_key) {
            args = input.substr(input.find_first_of(' ') + 1);
            break;
        }
        key = "";
    }
    if (i != _items.end()) {
        activate_item(key, args);
    } else {
        broker.show_menu(_description, _items);
    }
}

void MenuBackend::on_exit()
{
    if (_parent) {
        _parent->activate();
        _parent = nullptr;
    }
}

void MenuBackend::set_activated_callback(std::function<void(void)> f)
{
    _activated_callback = f;
}

void MenuBackend::activate()
{
    broker.set_active_menu(this);
    broker.show_menu(_description, _items);
    _activated_callback();
}

void MenuBackend::activate_item(std::string code, std::string args)
{
    if (_items.find(code) != _items.end()) {
        std::shared_ptr<MenuItem> i = _items.at(code);
        if (!i) {
            throw std::runtime_error("Trying to activate a null item");
        }
        if (i->type() == SUBMENU) {
            std::shared_ptr<MenuBackend> m = std::dynamic_pointer_cast<MenuBackend>(i);
            m->set_parent(this);
            m->execute(args);
        } else {
            i->execute(args);
            if (i->type() != EXIT || _parent) {
                broker.show_menu(_description, _items);
            }
        }
    }
}

void MenuBackend::set_parent(MenuBackend* parent)
{
    _parent = parent;
}
