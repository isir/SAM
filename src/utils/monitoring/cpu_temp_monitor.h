#ifndef CPUTEMPMONITOR_H
#define CPUTEMPMONITOR_H

#include "vc_based_monitor.h"

namespace Monitoring {

class CPUTempMonitor : public VCBasedMonitor
{
public:
    CPUTempMonitor();
    ~CPUTempMonitor() override;

    void update() override;
    std::string formatted_output() override;

private:
    double _temp;
};

}

#endif // CPUTEMPMONITOR_H
