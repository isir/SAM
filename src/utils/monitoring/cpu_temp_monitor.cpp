#include "cpu_temp_monitor.h"
#include <sstream>

namespace Monitoring {

CPUTempMonitor::CPUTempMonitor()
{
}

CPUTempMonitor::~CPUTempMonitor()
{
}

void CPUTempMonitor::update()
{
    std::string buf = vc_send_receive("measure_temp");
    unsigned int pos = buf.find('=') + 1;
    _temp = std::stod(buf.substr(pos, buf.size() - pos - 2));
}

std::string CPUTempMonitor::formatted_output()
{
    std::ostringstream out;
    out.precision(4);
    out << _temp << "°C";
    return out.str();
}

}
