#include "cpu_freq_monitor.h"
#include <sstream>

namespace Monitoring {

CPUFreqMonitor::CPUFreqMonitor()
    : _freq(0)
{
}

CPUFreqMonitor::~CPUFreqMonitor()
{
}

void CPUFreqMonitor::update()
{
    std::string buf = vc_send_receive("measure_clock arm");
    unsigned int pos = buf.find('=') + 1;
    _freq = std::stod(buf.substr(pos)) / 1000000.;
}

std::string CPUFreqMonitor::formatted_output()
{
    std::ostringstream out;
    out.precision(6);
    out << _freq << "MHz";
    return out.str();
}

}
