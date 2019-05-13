#ifndef CONSOLEMENUITEM_H
#define CONSOLEMENUITEM_H

#include <QString>
#include <functional>

class ConsoleMenuItem {
public:
    enum item_type_t {
        STANDARD,
        SUBMENU,
        EXIT,
        OTHER
    };

    ConsoleMenuItem() {}
    ConsoleMenuItem(QString description, QString code, std::function<void(QString)> callback, item_type_t type = STANDARD)
        : _callback(callback)
        , _type(type)
        , _description(description)
        , _code(code)
    {
    }

    item_type_t type() { return _type; }
    QString code() { return _code; }
    QString description() { return _description; }
    void execute(QString args = QString()) { _callback(args); }

protected:
    std::function<void(QString)> _callback;

    item_type_t _type;
    QString _description;
    QString _code;
};

#endif // CONSOLEMENUITEM_H
