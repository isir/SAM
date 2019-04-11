#include "consolemenu.h"
#include "consoleinput.h"
#include <QDebug>
#include <iostream>

QList<ConsoleMenu*> ConsoleMenu::_parents;
QList<QMetaObject::Connection> ConsoleMenu::_stream_connections;

ConsoleMenu::ConsoleMenu(QString title, QString code)
    : QObject(nullptr)
    , ConsoleMenuItem(
          title, code, [this](QString) { this->activate(); }, ConsoleMenuItem::SUBMENU)
    , _max_key_length(0)
    , _mqtt(MqttClient::instance())
{
    QObject::connect(this, &ConsoleMenu::ready_to_read, &ConsoleInput::instance(), &ConsoleInput::read_line);
    addItem(ConsoleMenuItem(
        "Exit", "exit", [this](QString) { this->on_exit(); }, ConsoleMenuItem::EXIT));
}

ConsoleMenu::~ConsoleMenu()
{
}

void ConsoleMenu::addItem(ConsoleMenuItem item)
{
    if (_menu.contains(item.code())) {
        qCritical() << "Cannot add \"" << item.description() << "\" because its key \"" << item.code() << "\" already exists.";
        return;
    }

    if (item.code().length() > _max_key_length) {
        _max_key_length = item.code().length();
    }
    _menu[item.code()] = item;
}

void ConsoleMenu::activate()
{
    static QMqttSubscription* sub = _mqtt.subscribe(QString("sam/menu/input"));

    foreach (QMetaObject::Connection con, _stream_connections) {
        QObject::disconnect(con);
    }

    if (_parents.isEmpty() || _parents.last() != this)
        _parents.push_back(this);

    _stream_connections.append(QObject::connect(&ConsoleInput::instance(), &ConsoleInput::new_line, this, &ConsoleMenu::on_user_input));
    _stream_connections.append(QObject::connect(sub, &QMqttSubscription::messageReceived, this, &ConsoleMenu::on_mqtt_message_received));
    display();
    emit activated();
    prepare_to_read();
}

void ConsoleMenu::prepare_to_read()
{
    std::cout << "> " << std::flush;
    emit ready_to_read();
}

void ConsoleMenu::display()
{
    QByteArray buffer("\r\n");
    QByteArray filler;
    if (!_description.isEmpty()) {
        filler.fill('-', _description.length() + 2);
        buffer.append(_description + " : \r\n");
        buffer.append(filler + "\r\n");
    }
    foreach (ConsoleMenuItem item, _menu) {
        filler.clear();
        filler.fill('.', _max_key_length - item.code().length() + 3);
        buffer.append("[" + item.code() + "]" + filler + " " + item.description() + "\r\n");
    }
    std::cout << buffer.toStdString() << std::flush;
    _mqtt.publish(QString("sam/menu/output"), buffer);
}

void ConsoleMenu::on_exit()
{
    if (_parents.size() > 1) {
        _parents.takeLast();
        _parents.last()->activate();
    }
    emit finished();
}

void ConsoleMenu::on_user_input(QString data)
{
    QString key, args;
    for (QMap<QString, ConsoleMenuItem>::key_iterator i = _menu.keyBegin(); i != _menu.keyEnd(); ++i) {
        key = (*i).split(' ', QString::SkipEmptyParts)[0];
        if (key == data) {
            break;
        } else if (data.startsWith(key + QString(" "))) {
            args = data.mid(key.length() + 1);
            break;
        }
    }

    if (_menu.contains(key)) {
        _menu[key].execute(args);
        if (_menu[key].type() != ConsoleMenuItem::SUBMENU && _menu[key].type() != ConsoleMenuItem::EXIT) {
            display();
            prepare_to_read();
        }
    } else {
        prepare_to_read();
    }
}

void ConsoleMenu::on_mqtt_message_received(const QMqttMessage msg)
{
    on_user_input(msg.payload());
}
