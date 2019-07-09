#ifndef REMOTECOMPUTERCONTROL_H
#define REMOTECOMPUTERCONTROL_H

#include "basic_controller.h"
#include "utils/sam.h"

class RemoteComputerControl : public BasicController {
public:
    explicit RemoteComputerControl(std::shared_ptr<SAM::Components> robot);
    ~RemoteComputerControl();

private:
    bool setup();
    void loop(double dt, double time);
    void cleanup();

    std::shared_ptr<SAM::Components> _robot;
};

#endif // REMOTECOMPUTERCONTROL_H
