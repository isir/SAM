#ifndef CPUFREQMONITOR_H
#define CPUFREQMONITOR_H

#include "vc_based_monitor.h"

namespace Monitoring {

class CPUFreqMonitor : public VCBasedMonitor
{
public:
    CPUFreqMonitor();
    ~CPUFreqMonitor() override;

    void update() override;
    std::string formatted_output() override;

private:
    double _freq;
};

}

#endif // CPUFREQMONITOR_H
