#include "menu_console.h"
#include <iostream>

MenuConsole::MenuConsole()
{
    connect_to_backend();

    QObject::connect(&_input, &ConsoleInput::new_line, this, &MenuConsole::input_received);
    QObject::connect(this, &MenuConsole::ready_to_read, &_input, &ConsoleInput::read_line);
    _input.moveToThread(&_thread);
    _thread.start();
}

MenuConsole::~MenuConsole()
{
    _thread.terminate();
}

void MenuConsole::show_menu_callback(QString title, QMap<QString, std::shared_ptr<MenuItem>> items)
{
    QByteArray buffer("\r\n");
    QByteArray filler;
    if (!title.isEmpty()) {
        filler.fill('-', title.length() + 1);
        buffer.append(title + ":\r\n");
        buffer.append(filler + "\r\n");
    }

    int max_key_length = 0;
    foreach (std::shared_ptr<MenuItem> item, items) {
        int length = item->code().length();
        if (length > max_key_length)
            max_key_length = length;
    }
    foreach (std::shared_ptr<MenuItem> item, items) {
        filler.clear();
        filler.fill('.', max_key_length - item->code().length() + 3);
        buffer.append("[" + item->code() + "]" + filler + " " + item->description() + "\r\n");
    }
    std::cout << buffer.toStdString() << std::flush;
    emit ready_to_read();
}
