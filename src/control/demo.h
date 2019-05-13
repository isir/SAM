#ifndef DEMO_H
#define DEMO_H

#include "basiccontroller.h"
#include "utils/sam.h"

class Demo : public BasicController {
public:
    Demo(SAM::Components robot, std::shared_ptr<QMqttClient> mqtt);
    ~Demo();

    bool setup();
    void loop(double dt, double time);
    void cleanup();

private:
    SAM::Components _robot;
};

#endif // DEMO_H
