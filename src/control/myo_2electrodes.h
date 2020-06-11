#ifndef MYO_2ELECTRODES_H
#define MYO_2ELECTRODES_H


#include "sam/sam.h"
#include "utils/threaded_loop.h"

#include <fstream>

class myo_2electrodes : public ThreadedLoop {
public:
    myo_2electrodes(std::shared_ptr<SAM::Components> robot);
    ~myo_2electrodes() override;

    void tare_IMU();
    void calibrations();

    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;

private:
    std::shared_ptr<SAM::Components> _robot;

    uint16_t _emg[2];
    static const uint16_t _n_electrodes = 6;
    int _th_low[_n_electrodes];
    int _th_high[_n_electrodes];
    std::ifstream _param_file;

    bool saveData = false;

    std::ofstream _file;
    bool _need_to_write_header;
    std::string _filename;
    clock::time_point _start_time;
};

#endif // MYO_2ELECTRODES_H
