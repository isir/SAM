#include "menu_mqtt.h"
#include "menu_backend.h"

MenuMQTT::MenuMQTT()
    : MenuFrontend()
{
    connect_to_backend();
    _mqtt.subscribe("sam/menu/input")->add_callback(this, [](std::string s) { MenuBackend::broker.handle_input(s); });
}

void MenuMQTT::show_menu(std::string title, std::map<std::string, std::shared_ptr<MenuItem>> items)
{
    std::string buffer;
    std::string filler;
    if (!title.empty()) {
        filler.insert(filler.begin(), title.length() + 1, '-');
        buffer.append(title + ":\r\n");
        buffer.append(filler + "\r\n");
    }
    for (auto item : items) {
        buffer.append("[" + item.second->code() + "]" + " " + item.second->description() + "\r\n");
    }
    _mqtt.publish("sam/menu/output", buffer, Mosquittopp::Client::QoS0, true);
}

void MenuMQTT::show_message(std::string msg)
{
    _mqtt.publish("sam/menu/output", msg, Mosquittopp::Client::QoS0, true);
}
