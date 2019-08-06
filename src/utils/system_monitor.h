#ifndef SYSTEMMONITOR_H
#define SYSTEMMONITOR_H

#include "control/threaded_loop.h"
#include "ui/mqtt_user.h"
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
};

#endif // SYSTEMMONITOR_H
