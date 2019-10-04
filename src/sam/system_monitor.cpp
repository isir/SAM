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
    _load_mon.update();
    _mqtt.publish("system/cpu_load", _load_mon.formatted_output());

    _temp_mon.update();
    _mqtt.publish("system/cpu_temp", _temp_mon.formatted_output());

    _freq_mon.update();
    _mqtt.publish("system/cpu_freq", _freq_mon.formatted_output());
}

void SystemMonitor::cleanup()
{
}
