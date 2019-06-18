#ifndef MENU_ITEM_H
#define MENU_ITEM_H

#include <QString>
#include <functional>

class MenuItem {
public:
    typedef enum {
        STANDARD,
        SUBMENU,
        EXIT,
        OTHER
    } ItemType;

    virtual ~MenuItem() {}

    void set_code(QString code) { _code = code; }
    QString code() { return _code; }

    void set_description(QString description) { _description = description; }
    QString description() { return _description; }

    ItemType type() { return _type; }
    void execute(QString args = QString()) { _callback(args); }

protected:
    MenuItem(QString code, QString description, std::function<void(QString)> callback, ItemType type = STANDARD)
        : _callback(callback)
        , _type(type)
        , _description(description)
        , _code(code)
    {
    }

    std::function<void(QString)> _callback;

    ItemType _type;
    QString _description;
    QString _code;
};

class StandardItem : public MenuItem {
public:
    StandardItem(QString code, QString description, std::function<void(QString)> callback)
        : MenuItem(code, description, callback, STANDARD)
    {
    }
};

class ExitItem : public MenuItem {
public:
    ExitItem(std::function<void(QString)> callback)
        : MenuItem("exit", "Exit", callback, EXIT)
    {
    }
};

#endif // MENU_ITEM_H
