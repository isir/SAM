#ifndef REMOTECOMPUTERCONTROL_H
#define REMOTECOMPUTERCONTROL_H

#include "threaded_loop.h"
#include "utils/sam.h"

class RemoteComputerControl : public ThreadedLoop {
public:
    explicit RemoteComputerControl(std::shared_ptr<SAM::Components> robot);
    ~RemoteComputerControl() override;

private:
    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;

    std::shared_ptr<SAM::Components> _robot;
};

#endif // REMOTECOMPUTERCONTROL_H
