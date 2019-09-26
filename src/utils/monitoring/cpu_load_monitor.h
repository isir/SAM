#ifndef CPULOADMONITOR_H
#define CPULOADMONITOR_H

#include "abstract_monitor.h"
#include <array>

namespace Monitoring {

class CPULoadMonitor : public AbstractMonitor
{
public:
    CPULoadMonitor();
    ~CPULoadMonitor() override;

    void update() override;
    std::string formatted_output() override;

private:
    static const int _ncpus = 4;
    std::array<int,_ncpus+1> _old_usr;
    std::array<int,_ncpus+1> _old_nice;
    std::array<int,_ncpus+1> _old_sys;
    std::array<int,_ncpus+1> _old_idle;
    std::array<double,_ncpus+1> _cpu_load;
};

}

#endif // CPULOADMONITOR_H
