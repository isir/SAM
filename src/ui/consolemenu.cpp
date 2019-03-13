#include "consolemenu.h"
#include "consoleinput.h"
#include <iostream>

QList<ConsoleMenu*> ConsoleMenu::_parents;
QList<QMetaObject::Connection> ConsoleMenu::_stream_connections;

ConsoleMenu::ConsoleMenu(QString title, QString code) : QObject(nullptr),
    ConsoleMenuItem(title, code, [this](QString){ this->activate(); }, ConsoleMenuItem::SUBMENU),
    _max_key_length(0)
{
    QObject::connect(this,&ConsoleMenu::ready_to_read,&ConsoleInput::instance(),&ConsoleInput::read_line);
    addItem(ConsoleMenuItem("Exit","exit",[this](QString){ this->on_exit(); }, ConsoleMenuItem::EXIT));
}

ConsoleMenu::~ConsoleMenu()
{

}

void ConsoleMenu::addItem(ConsoleMenuItem item) {
    if(_menu.contains(item.code())) {
        std::cerr << "Cannot add \"" << item.description().toStdString() << "\" because its key \"" << item.code().toStdString() << "\" already exists." << std::endl;
        return;
    }

    if(item.code().length() > _max_key_length) {
        _max_key_length = item.code().length();
    }
    _menu[item.code()] = item;
}

void ConsoleMenu::activate() {
    foreach(QMetaObject::Connection con, _stream_connections) {
        QObject::disconnect(con);
    }

    if(_parents.isEmpty() || _parents.last() != this)
        _parents.push_back(this);

    _stream_connections.append(QObject::connect(&ConsoleInput::instance(),&ConsoleInput::new_line,this,&ConsoleMenu::on_user_input));
    display();
    prepare_to_read();
}

void ConsoleMenu::prepare_to_read() {
    std::cout << "> ";
    std::cout.flush();
    emit ready_to_read();
}

void ConsoleMenu::display() {
    QString filler;
    std::cout << std::endl;
    if(!_description.isEmpty()) {
        filler.resize(_description.length() + 2 , '-');
        std::cout << _description.toStdString() << " : " << std::endl;
        std::cout << filler.toStdString() << std::endl;
    }
    foreach (ConsoleMenuItem item, _menu) {
        filler.clear();
        filler.resize(_max_key_length - item.code().length() + 3 , '.');
        std::cout << "[" << item.code().toStdString() << "]" << filler.toStdString() << " " << item.description().toStdString() << std::endl;
    }
    std::cout.flush();
}

void ConsoleMenu::on_exit() {
    if(_parents.size() > 1) {
        _parents.takeLast();
        _parents.last()->activate();
    }
    emit finished();
}

void ConsoleMenu::on_user_input(QString data) {
    QString key, args;
    for(QMap<QString,ConsoleMenuItem>::key_iterator i = _menu.keyBegin(); i != _menu.keyEnd(); ++i) {
        if(*i == data) {
            key = *i;
            break;
        }
        if(data.startsWith((*i) + QString(" "))) {
            key = *i;
            args = data.mid((*i).length() + 1);
            break;
        }
    }

    if(_menu.contains(key)) {
        _menu[key].execute(args);
        if(_menu[key].type() != ConsoleMenuItem::SUBMENU && _menu[key].type() != ConsoleMenuItem::EXIT) {
            display();
            prepare_to_read();
        }
    }
    else {
        prepare_to_read();
    }
}
