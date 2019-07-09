#ifndef MENU_BACKEND_H
#define MENU_BACKEND_H

#include "menu_broker.h"
#include "menu_item.h"
#include <QMap>
#include <QObject>
#include <QString>
#include <memory>

class MenuBackend : public QObject, public MenuItem {
    Q_OBJECT
public:
    MenuBackend(QString code = QString(), QString description = QString());
    ~MenuBackend();

    inline void add_item(QString code, QString description, std::function<void(QString)> callback)
    {
        add_item(std::make_shared<StandardItem>(code, description, callback));
    }
    void add_item(std::shared_ptr<MenuItem> item);

    template <typename T>
    void add_submenu_from_user(const std::unique_ptr<T>& obj)
    {
        if (obj) {
            add_item(obj->menu());
        }
    }

    static MenuBroker broker;
    void handle_input(QString input);

public slots:
    void activate();
    void activate_item(QString code, QString args = QString());

protected:
    virtual void on_exit();

private:
    void set_parent(MenuBackend* parent);
    MenuBackend* _parent;

    QMap<QString, std::shared_ptr<MenuItem>> _items;

signals:
    void activated();
    void finished();
    void show_menu(QString title, QMap<QString, std::shared_ptr<MenuItem>> items);
};

#endif // MENU_BACKEND_H
