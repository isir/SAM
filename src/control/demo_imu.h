#ifndef DEMOIMU_H
#define DEMOIMU_H

#include "control/algo/lawimu_wrist.h"
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
    void action_loop_imu();
    void action_loop_pb();

    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;

    std::shared_ptr<SAM::Components> _robot;

    clock::time_point _start_time;

    LawIMU_wrist _lawimu;
    int _cnt;
    double _lambdaW;
    double _thresholdW;
    int _pin_up; // pin buttons for open-loop
    int _pin_down; // pin buttons for open-loop
    int _pin_mode1; // 0 = open-loop with buttons; 1 = ergonomic with IMU
    int _pin_mode2; // 0 = open-loop with buttons; 1 = ergonomic with IMU
    int _pin_status; // 0 -> change status: start or stop
    bool _start; // indicates weither the loop is working
};

#endif // COMPENSATION_IMU_H
