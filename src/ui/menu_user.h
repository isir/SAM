#ifndef MENU_USER_H
#define MENU_USER_H

#include "menu_backend.h"

class MenuUser {
public:
    virtual ~MenuUser();

    std::shared_ptr<MenuBackend> menu();

protected:
    MenuUser(QString code = QString(), QString description = QString());

    std::shared_ptr<MenuBackend> _menu;
};

#endif // MENU_USER_H
