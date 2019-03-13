#ifndef FACTORY_H
#define FACTORY_H

#include <QMap>
#include <QString>

#include "server.h"

namespace RoboClaw {
class Factory
{
public:
     static Server* get(QString port_name, int baudrate);
     static void cleanup();

private:
    Factory();

    static QMap<QString,Server*> _map;
};
}

#endif // FACTORY_H
