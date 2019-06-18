#include "menu_backend.h"

MenuBroker MenuBackend::broker;

MenuBackend::MenuBackend(QString code, QString description)
    : MenuItem(
        code, description, [this](QString) { activate(); }, SUBMENU)
    , _parent(nullptr)
{
    add_item("exit", "Exit", [this](QString) { on_exit(); });
}

MenuBackend::~MenuBackend()
{
}

void MenuBackend::add_item(std::shared_ptr<MenuItem> item)
{
    _items.insert(item->code(), item);
}

void MenuBackend::handle_input(QString input)
{
    QString key, args;
    for (QMap<QString, std::shared_ptr<MenuItem>>::key_iterator i = _items.keyBegin(); i != _items.keyEnd(); ++i) {
        key = (*i).split(' ', QString::SkipEmptyParts)[0];
        if (key == input) {
            break;
        } else if (input.startsWith(key + QString(" "))) {
            args = input.mid(key.length() + 1);
            break;
        }
        key = "";
    }

    activate_item(key, args);
}

void MenuBackend::on_exit()
{
    if (_parent) {
        _parent->activate();
    }
    _parent = nullptr;
    emit finished();
}

void MenuBackend::activate()
{
    broker.set_active_menu(this);
    emit show_menu(_description, _items);
    emit activated();
}

void MenuBackend::activate_item(QString code, QString args)
{
    if (_items.contains(code)) {
        std::shared_ptr<MenuItem> i = _items.value(code);
        if (!i) {
            throw std::runtime_error("Trying to activate a null item");
        }

        if (i->type() == SUBMENU) {
            std::shared_ptr<MenuBackend> m = std::dynamic_pointer_cast<MenuBackend>(i);
            m->set_parent(this);
        }
        i->execute(args);
    }
}

void MenuBackend::set_parent(MenuBackend* parent)
{
    _parent = parent;
}
