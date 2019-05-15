#ifndef FACTORY_H
#define FACTORY_H

#include <QMap>
#include <QString>

#include "utils/serial_port.h"

namespace RC {
class Factory {
public:
    static std::shared_ptr<SerialPort> get(QString port_name, unsigned int baudrate);

private:
    Factory();

    static QMap<QString, std::shared_ptr<SerialPort>> _map;
};
}

#endif // FACTORY_H
