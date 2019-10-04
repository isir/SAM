#ifndef ABSTRACTMONITOR_H
#define ABSTRACTMONITOR_H

#include <string>

namespace Monitoring {

class AbstractMonitor
{
public:
    AbstractMonitor();
    virtual ~AbstractMonitor();

    virtual void update() = 0;
    virtual std::string formatted_output() = 0;
};

}

#endif // ABSTRACTMONITOR_H
