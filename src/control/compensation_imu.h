#ifndef COMPENSATIONIMU_H
#define COMPENSATIONIMU_H

#include "control/algo/lawimu_wrist.h"
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
    void displayPin();

    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;

    std::shared_ptr<SAM::Components> _robot;
    Socket _receiver;

    clock::time_point _start_time;

    std::ofstream _file;
    bool _need_to_write_header;
    bool saveData = true;

    int _cnt;
    uint16_t _emg[2];
    int buzzN, loopN;
    static const uint16_t _n_electrodes = 6;
    int _th_low[_n_electrodes];
    int _th_high[_n_electrodes];
    std::ifstream _param_file;

    LawIMU_wrist _lawimu;

    Param<double> _lambdaW;
    Param<double> _thresholdW;
};

#endif // COMPENSATION_IMU_H
