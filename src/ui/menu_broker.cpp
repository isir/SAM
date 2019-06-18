#include "menu_broker.h"
#include "menu_backend.h"

MenuBroker::MenuBroker(QObject* parent)
    : QObject(parent)
    , _active_menu(nullptr)
{
}

void MenuBroker::set_active_menu(MenuBackend* menu)
{
    static QMetaObject::Connection outbound;
    QObject::disconnect(outbound);
    outbound = QObject::connect(menu, &MenuBackend::show_menu, this, &MenuBroker::show_menu);
    _active_menu = menu;
}

void MenuBroker::handle_input(QString input)
{
    if (_active_menu) {
        _active_menu->handle_input(input);
    }
}
