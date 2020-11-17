#ifndef CYBATHLON_H
#define CYBATHLON_H

#include "control/algo/lawimu_wrist.h"
#include "sam/sam.h"
#include "utils/interfaces/mqtt_user.h"
#include "utils/threaded_loop.h"
#include <fstream>

class Cybathlon : public ThreadedLoop, public MqttUser  {
public:
    explicit Cybathlon(std::shared_ptr<SAM::Components> robot);
    ~Cybathlon() override;

    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;

    void init_IMU();

private:
    void readAllADC();
    void processQuantumHand(int emg1, int emg2, int16_t btn_posture);
    void changeFilter();

    std::shared_ptr<SAM::Components> _robot;

    clock::time_point _start_time;
    int _cnt_loop;

    int _cnt_imu;
    LawIMU_wrist _lawimu;

    Param<int> _lambdaW;
    Param<double> _thresholdW;

    std::ifstream _param_file;
    static const uint16_t _n_electrodes = 6;
    int _th[_n_electrodes];
    uint16_t _electrodes[_n_electrodes];

    static const uint16_t _order_filter = 3;
    double _filter_coef[_order_filter+1];
    bool _filter = 1;

    std::ofstream _file;
};

#endif // CYBATHLON_H
