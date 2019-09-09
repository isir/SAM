#include "system_monitor.h"
#include <iterator>
#include <sstream>

SystemMonitor::SystemMonitor()
    : ThreadedLoop("system_monitor", 1)
{
}

SystemMonitor::~SystemMonitor()
{
    stop_and_join();
}

bool SystemMonitor::setup()
{
    return true;
}

void SystemMonitor::loop(double, clock::time_point)
{
    static int old_usr[5] = { 0 }, old_nice[5] = { 0 }, old_sys[5] = { 0 }, old_idle[5] = { 0 };

    double cpu_load[5];

    std::string buf;
    std::string msg;

    std::ifstream stat_file("/proc/stat");
    std::ifstream temp_file("/sys/class/thermal/thermal_zone0/temp");

    for (unsigned int i = 0; i < 5; ++i) {
        std::getline(stat_file, buf);
        std::istringstream ss(buf);
        std::vector<std::string> infos((std::istream_iterator<std::string>(ss)), std::istream_iterator<std::string>());

        if (infos.size() > 5) {
            int usr = std::stoi(infos[1]);
            int nice = std::stoi(infos[2]);
            int sys = std::stoi(infos[3]);
            int idle = std::stoi(infos[4]);
            cpu_load[i] = 1. - static_cast<double>(idle - old_idle[i]) / (usr + nice + sys + idle - old_usr[i] - old_nice[i] - old_sys[i] - old_idle[i]);
            old_usr[i] = usr;
            old_nice[i] = nice;
            old_sys[i] = sys;
            old_idle[i] = idle;
        }
        msg.append(std::to_string(cpu_load[i]) + " ");
    }

    std::getline(temp_file, buf);

    _mqtt.publish("system/cpu_load", msg);
    _mqtt.publish("system/cpu_temp", buf);
}

void SystemMonitor::cleanup()
{
}
