#ifndef DEMO_H
#define DEMO_H

#include "sam/sam.h"
#include "utils/threaded_loop.h"

class Demo : public ThreadedLoop {
public:
    Demo(std::shared_ptr<SAM::Components> robot);
    ~Demo() override;

    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;

private:
    std::shared_ptr<SAM::Components> _robot;
};

#endif // DEMO_H
