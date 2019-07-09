#ifndef MENU_BROKER_H
#define MENU_BROKER_H

#include "menu_item.h"
#include <QMap>
#include <QObject>
#include <QString>
#include <memory>

class MenuBackend;

class MenuBroker : public QObject {
    Q_OBJECT
public:
    explicit MenuBroker(QObject* parent = nullptr);

    void set_active_menu(MenuBackend* menu);

public slots:
    void handle_input(QString input);

private:
    MenuBackend* _active_menu;

signals:
    void show_menu(QString title, QMap<QString, std::shared_ptr<MenuItem>> items);
};

#endif // MENU_BROKER_H
