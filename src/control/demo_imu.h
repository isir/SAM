#ifndef DEMOIMU_H
#define DEMOIMU_H

#include "control/algo/lawimu.h"
#include "sam/sam.h"
#include "utils/socket.h"
#include "utils/threaded_loop.h"
#include <fstream>

class DemoIMU : public ThreadedLoop {
public:
    explicit DemoIMU(std::shared_ptr<SAM::Components> robot);
    ~DemoIMU() override;

private:
    void tare_IMU();
    void displayPin();

    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;

    std::shared_ptr<SAM::Components> _robot;

    clock::time_point _start_time;

    LawIMU _lawimu;
    int _cnt;
    int _lambdaW;
    double _thresholdW;
    int _pin_up;
    int _pin_down;
};

#endif // COMPENSATION_IMU_H
