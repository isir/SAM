#ifndef VOLUNTARYCONTROL_H
#define VOLUNTARYCONTROL_H

#include "sam/sam.h"
#include "utils/threaded_loop.h"
#include <fstream>

class VoluntaryControl : public ThreadedLoop {
public:
    explicit VoluntaryControl(std::shared_ptr<SAM::Components> robot);
    ~VoluntaryControl() override;

private:
    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;

    std::ofstream _file;

    std::shared_ptr<SAM::Components> _robot;
    bool _need_to_write_header;
};

#endif // VOLUNTARYCONTROL_H
