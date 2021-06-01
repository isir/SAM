#ifndef FACTORY_H
#define FACTORY_H

#include <map>
#include <string>
#include <memory>
#include "utils/serial_port.h"

namespace RC {
class Factory {
public:
    static std::shared_ptr<SerialPort> get(std::string port_name, unsigned int baudrate);

private:
    Factory();

    static std::map<std::string, std::shared_ptr<SerialPort>> _map;
};
}

#endif // FACTORY_H
