#ifndef COMPENSATIONIMU_H
#define COMPENSATIONIMU_H

#include "control/algo/lawimu_we.h"
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
    void law_elbowAndWrist(int init_cnt, double debugData[], double beta);
    void law_wristOnly(int init_cnt, double debugData[]);

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
    LawIMU_WE _lawimuWE;

    bool protoCyb = true;

    Param<double> _lambdaW;
    Param<double> _thresholdW;
    Param<double> _lambdaE;
    Param<double> _thresholdE;
    Param<int> _lt; // length of the trunk
    Param<int> _lua; // upper-arm length
    Param<int> _lfa; // forearm length
    Param<int> _lsh; // from head to shoulder length

    Eigen::Quaterniond qWhite_record, qYellow_record;
};

#endif // COMPENSATION_IMU_H
