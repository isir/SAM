#include "menu_console.h"
#include "ux/menu/menu_backend.h"
#include <fcntl.h>
#include <iostream>
#include <unistd.h>

MenuConsole::MenuConsole()
    : Worker("read_line")
{
    connect_to_backend();
    if (fcntl(0, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK) < 0)
        std::cerr << "fcntl failed with error " << errno << std::endl;
}

MenuConsole::~MenuConsole()
{
    stop();
    fcntl(0, F_SETFL, fcntl(0, F_GETFL) & (~O_NONBLOCK));
}

void MenuConsole::show_menu(std::string title, std::map<std::string, std::shared_ptr<MenuItem>> items)
{
    std::string buffer("\r\n");
    std::string filler;
    if (title.length() > 0) {
        filler.insert(0, title.length() + 1, '-');
        buffer.append(title + ":\r\n");
        buffer.append(filler + "\r\n");
    }

    std::size_t max_key_length = 0;
    for (auto item : items) {
        std::size_t length = item.second->code().length();
        if (length > max_key_length)
            max_key_length = length;
    }
    for (auto item : items) {
        filler.clear();
        filler.insert(0, max_key_length - item.second->code().length() + 3, '.');
        buffer += "[" + item.second->code() + "]" + filler + " " + item.second->description() + "\r\n";
    }
    std::cout << buffer << std::flush;
    std::cout << "> " << std::flush;

    do_work();
}

void MenuConsole::show_message(std::string msg)
{
    std::cout << msg << std::endl;
}

void MenuConsole::work()
{
    char buffer[128];
    int n = 0;

    while (true) {
        n = read(0, buffer, 128);
        if (n > 0 || !_worker_loop_condition) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (n > 0) {
        MenuBackend::broker.handle_input(std::string(buffer, buffer + n - 1));
    }
}
