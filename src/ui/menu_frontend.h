#ifndef MENUFRONTEND_H
#define MENUFRONTEND_H

#include "menu_item.h"
#include <QMap>
#include <QObject>
#include <memory>

class MenuFrontend : public QObject {
    Q_OBJECT
public:
    MenuFrontend();
    virtual ~MenuFrontend();

public slots:
    virtual void show_menu_callback(QString title, QMap<QString, std::shared_ptr<MenuItem>> items) = 0;

protected:
    void connect_to_backend();

signals:
    void input_received(QString input);
};

#endif // MENUFRONTEND_H
