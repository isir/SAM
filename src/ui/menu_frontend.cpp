#include "menu_frontend.h"
#include "menu_backend.h"

MenuFrontend::MenuFrontend()
{
}

MenuFrontend::~MenuFrontend()
{
}

void MenuFrontend::connect_to_backend()
{
    MenuBackend::broker.register_frontend(this);
}
