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
    QObject::connect(&MenuBackend::broker, &MenuBroker::show_menu, this, &MenuFrontend::show_menu_callback);
    QObject::connect(this, &MenuFrontend::input_received, &MenuBackend::broker, &MenuBroker::handle_input);
}
