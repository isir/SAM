#include "menu_user.h"

MenuUser::MenuUser(QString code, QString description)
    : _menu(std::make_shared<MenuBackend>(code, description))
{
}

MenuUser::~MenuUser()
{
}

std::shared_ptr<MenuBackend> MenuUser::menu()
{
    return _menu;
}
