#include "menu_user.h"

MenuUser::MenuUser(std::string code, std::string description, std::function<void(void)> exit_callback)
    : _menu(std::make_shared<MenuBackend>(code, description, exit_callback))
{
}

MenuUser::~MenuUser()
{
}

std::shared_ptr<MenuBackend> MenuUser::menu()
{
    return _menu;
}
