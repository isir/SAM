#ifndef REMOTECOMPUTERCONTROL_H
#define REMOTECOMPUTERCONTROL_H

#include "basiccontroller.h"
#include "ui/consolemenu.h"
#include "utils/sam.h"
#include "utils/settings.h"

class RemoteComputerControl : public BasicController {
public:
    explicit RemoteComputerControl(SAM::Components robot, std::shared_ptr<QMqttClient> mqtt);
    ~RemoteComputerControl();

private:
    bool setup();
    void loop(double dt, double time);
    void cleanup();

    SAM::Components _robot;
    Settings _settings;
};

#endif // REMOTECOMPUTERCONTROL_H
