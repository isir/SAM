#include "menu_item.h"

MenuItem::MenuItem(std::string code, std::string description, std::function<void(std::string)> callback, ItemType type)
    : _callback(callback)
    , _type(type)
    , _description(description)
    , _code(code)
{
}

MenuItem::~MenuItem()
{
}

StandardItem::StandardItem(std::string code, std::string description, std::function<void(std::string)> callback)
    : MenuItem(code, description, callback, STANDARD)
{
}

StandardItem::~StandardItem()
{
}

ExitItem::ExitItem(std::function<void(std::string)> callback)
    : MenuItem("exit", "Exit", callback, EXIT)
{
}

ExitItem::~ExitItem()
{
}
