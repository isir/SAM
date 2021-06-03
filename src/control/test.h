#ifndef TEST_H
#define TEST_H

#include "sam/sam.h"
#include "utils/interfaces/mqtt_user.h"
#include "utils/threaded_loop.h"
#include <fstream>

class Test : public ThreadedLoop, public MqttUser
{
public:
    Test(std::shared_ptr<SAM::Components> robot);
    ~Test() override;

    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;

private:
    std::shared_ptr<SAM::Components> _robot;
    clock::time_point _start_time;

    // indicate whether to save data
    bool saveData = true;
    std::ofstream _file;
};

#endif // TEST_H
