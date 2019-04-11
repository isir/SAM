#ifndef CONSOLEMENU_H
#define CONSOLEMENU_H

#include "consolemenuitem.h"
#include "utils/mqttclient.h"
#include <QList>
#include <QMap>
#include <QObject>

class ConsoleMenu : public QObject, public ConsoleMenuItem {
    Q_OBJECT
public:
    explicit ConsoleMenu(QString title = QString(), QString code = QString());
    ~ConsoleMenu();

    void addItem(ConsoleMenuItem item);
    void set_title(QString title) { _description = title; }
    void set_code(QString title) { _code = title; }

public slots:
    virtual void activate();

protected:
    void prepare_to_read();

    static QList<ConsoleMenu*> _parents;
    QMap<QString, ConsoleMenuItem> _menu;
    int _max_key_length;
    MqttClient& _mqtt;

protected slots:
    void display();
    virtual void on_exit();
    void on_user_input(QString data);
    void on_mqtt_message_received(const QMqttMessage msg);

private:
    static QList<QMetaObject::Connection> _stream_connections;

signals:
    void activated();
    void ready_to_read();
    void finished();
};

#endif // CONSOLEMENU_H
