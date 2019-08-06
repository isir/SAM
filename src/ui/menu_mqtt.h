#ifndef MENU_MQTT_H
#define MENU_MQTT_H

#include "menu_frontend.h"
#include "mqtt_user.h"

class MenuMQTT : public MenuFrontend, public MqttUser {
public:
    MenuMQTT();

    void show_menu(std::string title, std::map<std::string, std::shared_ptr<MenuItem>> items) override;
    void show_message(std::string msg) override;
};

#endif // MENU_MQTT_H
