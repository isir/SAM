#ifndef MENU_ITEM_H
#define MENU_ITEM_H

#include <functional>
#include <string>

class MenuItem {
public:
    typedef enum {
        STANDARD,
        SUBMENU,
        EXIT,
        OTHER
    } ItemType;

    virtual ~MenuItem();

    inline void set_code(std::string code) { _code = code; }
    inline std::string code() { return _code; }
    inline void set_description(std::string description) { _description = description; }
    inline std::string description() { return _description; }
    inline ItemType type() { return _type; }

    void execute(std::string args = std::string()) { _callback(args); }

protected:
    MenuItem(std::string code, std::string description, std::function<void(std::string)> callback, ItemType type = STANDARD);

    std::function<void(std::string)> _callback;

    ItemType _type;
    std::string _description;
    std::string _code;
};

class StandardItem : public MenuItem {
public:
    StandardItem(std::string code, std::string description, std::function<void(std::string)> callback);
    ~StandardItem();
};

class ExitItem : public MenuItem {
public:
    ExitItem(std::function<void(std::string)> callback);
    ~ExitItem();
};

#endif // MENU_ITEM_H
