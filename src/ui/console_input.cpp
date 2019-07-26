#include "console_input.h"
#include <iostream>
#include <stdio.h>

ConsoleInput::ConsoleInput(QObject* parent)
    : QObject(parent)
{
}

ConsoleInput::~ConsoleInput()
{
}

void ConsoleInput::read_line()
{
    std::cout << "> " << std::flush;
    std::cin.getline(&_buffer[0], _buffer_size, '\n');
    emit new_line(QString(_buffer));
}
