#ifndef DEMO_H
#define DEMO_H

#include "threaded_loop.h"
#include "utils/sam.h"
#include "utils/settings.h"

class Demo : public ThreadedLoop {
public:
    Demo(std::shared_ptr<SAM::Components> robot);
    ~Demo();

    bool setup();
    void loop(double dt, double time);
    void cleanup();

private:
    std::shared_ptr<SAM::Components> _robot;

    Settings _settings;
    int _pin_up;
    int _pin_down;
};

#endif // DEMO_H
