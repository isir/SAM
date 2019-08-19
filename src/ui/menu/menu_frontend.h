#ifndef MENU_FRONTEND_H
#define MENU_FRONTEND_H

#include "ux/menu/menu_item.h"
#include <map>
#include <memory>

class MenuFrontend {
public:
    MenuFrontend();
    virtual ~MenuFrontend();

    virtual void show_menu(std::string title, std::map<std::string, std::shared_ptr<MenuItem>> items) = 0;
    virtual void show_message(std::string msg) = 0;

protected:
    void connect_to_backend();
};

#endif // MENU_FRONTEND_H
