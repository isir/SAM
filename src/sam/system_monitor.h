#ifndef SYSTEMMONITOR_H
#define SYSTEMMONITOR_H

#include "utils/interfaces/mqtt_user.h"
#include "utils/monitoring/cpu_freq_monitor.h"
#include "utils/monitoring/cpu_load_monitor.h"
#include "utils/monitoring/cpu_temp_monitor.h"
#include "utils/threaded_loop.h"
#include <fstream>
#include <memory>

class SystemMonitor : public ThreadedLoop, public MqttUser {
public:
    explicit SystemMonitor();
    ~SystemMonitor() override;

private:
    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;

    Monitoring::CPUFreqMonitor _freq_mon;
    Monitoring::CPULoadMonitor _load_mon;
    Monitoring::CPUTempMonitor _temp_mon;
};

#endif // SYSTEMMONITOR_H
