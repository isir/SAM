#include "factory.h"

QMap<QString,RoboClaw::Server*> RoboClaw::Factory::_map;

RoboClaw::Factory::Factory()
{

}

RoboClaw::Server* RoboClaw::Factory::get(QString port_name, int baudrate) {
    if(!_map.contains(port_name)) {
        return _map.insert(port_name,new Server(port_name,baudrate)).value();
    }
    if(_map.value(port_name)->baudrate() != baudrate) {
        return nullptr;
    }
    return _map.value(port_name);
}

void RoboClaw::Factory::cleanup() {
    foreach (Server* s, _map) {
        delete s;
    }
    _map.clear();
}
