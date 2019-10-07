#include "cpu_load_monitor.h"
#include <fstream>
#include <iterator>
#include <sstream>
#include <vector>

namespace Monitoring {

CPULoadMonitor::CPULoadMonitor()
{
    _old_usr.fill(0);
    _old_nice.fill(0);
    _old_sys.fill(0);
    _old_idle.fill(0);
    _cpu_load.fill(0);
}

CPULoadMonitor::~CPULoadMonitor()
{
}

void CPULoadMonitor::update()
{
    std::ifstream stat_file("/proc/stat");
    std::string buf;

    for (unsigned int i = 0; i < _ncpus + 1; ++i) {
        std::getline(stat_file, buf);
        std::istringstream ss(buf);
        std::vector<std::string> infos((std::istream_iterator<std::string>(ss)), std::istream_iterator<std::string>());

        if (infos.size() > 5) {
            int usr = std::stoi(infos[1]);
            int nice = std::stoi(infos[2]);
            int sys = std::stoi(infos[3]);
            int idle = std::stoi(infos[4]);
            _cpu_load[i] = 1. - static_cast<double>(idle - _old_idle[i]) / (usr + nice + sys + idle - _old_usr[i] - _old_nice[i] - _old_sys[i] - _old_idle[i]);
            _old_usr[i] = usr;
            _old_nice[i] = nice;
            _old_sys[i] = sys;
            _old_idle[i] = idle;
        }
    }
}

std::string CPULoadMonitor::formatted_output()
{
    std::string ret;
    for (unsigned int i = 0; i < _ncpus + 1; ++i) {
        ret += std::to_string(_cpu_load[i]) + " ";
    }
    ret.pop_back();
    return ret;
}

}
