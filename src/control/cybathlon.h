#ifndef CYBATHLON_H
#define CYBATHLON_H

#include "control/algo/lawimu_wrist.h"
#include "sam/sam.h"
#include "utils/interfaces/mqtt_user.h"
#include "utils/threaded_loop.h"
#include <fstream>

class Cybathlon : public ThreadedLoop, public MqttUser {
public:
    explicit Cybathlon(std::shared_ptr<SAM::Components> robot);
    ~Cybathlon() override;

    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;

    void tare_IMU();
    void displayPin();

private:
    void readAllADC();

    std::shared_ptr<SAM::Components> _robot;

    clock::time_point _start_time;

    int _cnt;
    LawIMU_wrist _lawimu;

    Param<int> _lambdaW;
    Param<double> _thresholdW;

    std::ifstream _param_file;
    static const uint16_t _n_electrodes = 6;
    int _th_low[_n_electrodes];
    int _th_high[_n_electrodes];
    uint16_t _electrodes[_n_electrodes];
};

#endif // CYBATHLON_H
