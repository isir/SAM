#include "pronosupination.h"
#include "peripherals/roboclaw/factory.h"
#include "peripherals/roboclaw/server.h"
#include <iostream>

PronoSupination::PronoSupination() : RoboClaw::Client()
{
    _settings.beginGroup("Pronosupination");
    int address = _settings.value("address", 0x80).toInt();
    Channel channel = static_cast<Channel>(_settings.value("channel",2).toInt());
    QString port_name = _settings.value("port_name","/dev/ttyAMA0").toString();
    int baudrate = _settings.value("baudrate",230400).toInt();

    set_address(address,channel);
    RoboClaw::Server *s = RoboClaw::Factory::get(port_name,baudrate);
    if(s != nullptr) {
        s->register_client(this);
    }
    else {
        std::cerr << "Failed to obtain server object." << std::endl;
    }

    QObject::connect(&_menu,&ConsoleMenu::finished,this,&PronoSupination::on_exit);

    _menu.set_title(QString("Pronosupination - ") + read_firmware_version() + " - " + port_name + " - " + QString::number(baudrate) + " - " + QString::number(address) + "/" + QString::number(chan()));
    _menu.set_code(QString("pronosup"));

    _menu.addItem(ConsoleMenuItem("Forward","f",[this](QString){ this->forward(64); }));
    _menu.addItem(ConsoleMenuItem("Backward","b",[this](QString){ this->backward(64); }));
    _menu.addItem(ConsoleMenuItem("Print current","pc",[this](QString){ std::cout << "Current: " << this->read_current() << "A" << std::endl; }));
    _menu.addItem(ConsoleMenuItem("Stop","s",[this](QString){ this->forward(0); }));
    _menu.addItem(ConsoleMenuItem("Go to","g",[this](QString args){ this->move_to(6000, 3000, 6000, args.toInt()); }));
    _menu.addItem(ConsoleMenuItem("Read encoder","e",[this](QString){ std::cout << this->read_encoder_position() << std::endl; }));
    _menu.addItem(ConsoleMenuItem("Set encoder zero","z",[this](QString){ this->set_encoder_position(0); }));
    _menu.addItem(ConsoleMenuItem("Read PID","p",[this](QString){ this->read_position_pid(); }));

    RoboClaw::velocity_pid_params_t v_params = {};
    _settings.beginGroup("Velocity_PID");
    v_params.p = _settings.value("kp",4.2).toDouble();
    v_params.i = _settings.value("ki",0.56).toDouble();
    v_params.d = _settings.value("kd",0.).toDouble();
    v_params.qpps = _settings.value("qpps",6000).toUInt();
    _settings.endGroup();
    set_velocity_pid(v_params);

    RoboClaw::position_pid_params_t p_params = {};
    _settings.beginGroup("Position_PID");
    p_params.p = _settings.value("kp",51.).toDouble();
    p_params.i = _settings.value("ki",1.4).toDouble();
    p_params.d = _settings.value("kd",428.).toDouble();
    p_params.i_max = _settings.value("i_max",60.).toDouble();
    p_params.deadzone = _settings.value("deadzone",0).toInt();
    p_params.min_pos = _settings.value("min_pos",-15000).toInt();
    p_params.max_pos = _settings.value("max_pos",15000).toInt();
    _settings.endGroup();
    set_position_pid(p_params);
}

PronoSupination::~PronoSupination() {

}

PronoSupination& PronoSupination::instance() {
    static PronoSupination _instance;
    return _instance;
}

ConsoleMenu& PronoSupination::menu() {
    return _menu;
}

void PronoSupination::on_exit() {
    forward(0);
}
