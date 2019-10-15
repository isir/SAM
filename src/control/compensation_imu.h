#ifndef COMPENSATIONIMU_H
#define COMPENSATIONIMU_H

#include "control/algo/lawimu.h"
#include "sam/sam.h"
#include "utils/socket.h"
#include "utils/threaded_loop.h"
#include <fstream>

class CompensationIMU : public ThreadedLoop {
public:
    explicit CompensationIMU(std::shared_ptr<SAM::Components> robot);
    ~CompensationIMU() override;

private:
    void tare_IMU();
    void receiveData();
    void displayPin();

    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;

    std::shared_ptr<SAM::Components> _robot;
    Socket _receiver;

    clock::time_point _start_time;

    std::ofstream _file;
    bool _need_to_write_header;
    int _cnt;
    LawIMU _lawimu;

    int _Lt;
    double _Lua;
    double _Lfa;
    double _l;
    int _lambdaW, _lambda;
    double _thresholdW, _threshold;
};

#endif // COMPENSATION_IMU_H
