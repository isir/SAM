#include "console_input.h"
#include <iostream>
#include <stdio.h>

ConsoleInput::ConsoleInput(QObject* parent)
    : QObject(parent)
{
    moveToThread(&_thread);
    _thread.start();
}

ConsoleInput::~ConsoleInput()
{
    _thread.quit();
    _thread.wait();
}

void ConsoleInput::read_line()
{
    std::cin.getline(&_buffer[0], _buffer_size, '\n');
    emit new_line(QString(_buffer));
}
