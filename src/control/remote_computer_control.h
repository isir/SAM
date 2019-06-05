#ifndef REMOTECOMPUTERCONTROL_H
#define REMOTECOMPUTERCONTROL_H

#include "basic_controller.h"
#include "ui/console_menu.h"
#include "utils/sam.h"
#include "utils/settings.h"

class RemoteComputerControl : public BasicController {
public:
    explicit RemoteComputerControl(SAM::Components robot);
    ~RemoteComputerControl();

private:
    bool setup();
    void loop(double dt, double time);
    void cleanup();

    SAM::Components _robot;
    Settings _settings;
};

#endif // REMOTECOMPUTERCONTROL_H
