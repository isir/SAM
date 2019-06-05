#ifndef DEMO_H
#define DEMO_H

#include "basic_controller.h"
#include "utils/sam.h"
#include "utils/settings.h"

class Demo : public BasicController {
public:
    Demo(SAM::Components robot);
    ~Demo();

    bool setup();
    void loop(double dt, double time);
    void cleanup();

private:
    SAM::Components _robot;

    Settings _settings;
    int _pin_up;
    int _pin_down;
};

#endif // DEMO_H
