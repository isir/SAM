#include "factory.h"

std::map<std::string, std::shared_ptr<SerialPort>> RC::Factory::_map;

RC::Factory::Factory()
{
}

std::shared_ptr<SerialPort> RC::Factory::get(std::string port_name, unsigned int baudrate)
{
    if (_map.find(port_name) == _map.end()) {
        std::shared_ptr<SerialPort> sp = std::make_shared<SerialPort>(port_name, baudrate);
        sp->open();
        _map.emplace(port_name, sp);
        return sp;
    } else if (_map.at(port_name)->baudrate() != baudrate) {
        throw std::runtime_error("Requested port is already in use with a different baudrate (" + std::to_string(_map.at(port_name)->baudrate()) + " - requested " + std::to_string(baudrate) + ").");
    }
    return _map.at(port_name);
}
