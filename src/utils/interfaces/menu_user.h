#ifndef MENU_USER_H
#define MENU_USER_H

#include "ux/menu/menu_backend.h"

class MenuUser {
public:
    virtual ~MenuUser();

    std::shared_ptr<MenuBackend> menu();

protected:
    MenuUser(std::string code = std::string(), std::string description = std::string(), std::function<void(void)> exit_callback = std::function<void(void)>());

    std::shared_ptr<MenuBackend> _menu;
};

#endif // MENU_USER_H
