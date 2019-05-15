#include "factory.h"

QMap<QString, std::shared_ptr<SerialPort>> RC::Factory::_map;

RC::Factory::Factory()
{
}

std::shared_ptr<SerialPort> RC::Factory::get(QString port_name, unsigned int baudrate)
{
    if (!_map.contains(port_name)) {
        return _map.insert(port_name, std::make_shared<SerialPort>(port_name, baudrate)).value();
    }
    if (_map.value(port_name)->baudrate() != baudrate) {
        throw std::runtime_error("Requested port is already in use with a different baudrate.");
        return nullptr;
    }
    return _map.value(port_name);
}
