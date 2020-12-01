#ifndef TESTIMU_H
#define TESTIMU_H

#include "algo/lawjacobian.h"
#include "sam/sam.h"
#include "utils/socket.h"
#include "utils/interfaces/mqtt_user.h"
#include "utils/threaded_loop.h"
#include <fstream>

class TestIMU : public ThreadedLoop, public MqttUser  {
public:
    explicit TestIMU(std::shared_ptr<SAM::Components> robot);
    ~TestIMU() override;

private:
    void tare_IMU();
    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;

    std::shared_ptr<SAM::Components> _robot;
    Socket _receiver;
    std::ofstream _file;
    int _cnt;
    clock::time_point _start_time;

};

#endif // TESTIMU_H
