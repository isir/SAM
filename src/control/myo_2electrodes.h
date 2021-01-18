#ifndef MYO_2ELECTRODES_H
#define MYO_2ELECTRODES_H

#include "sam/sam.h"
#include "utils/threaded_loop.h"

#include <fstream>

class myo_2electrodes : public ThreadedLoop, public MqttUser {
public:
    myo_2electrodes(std::shared_ptr<SAM::Components> robot);
    ~myo_2electrodes() override;

    void tare_allIMU();
    void tare_whiteIMU();
    void tare_yellowIMU();
    void tare_redIMU();
    void elbowTo90();
    void calibrations();
    void readAllADC();

    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;

private:
    std::shared_ptr<SAM::Components> _robot;

    uint16_t _electrodes[6];
    static const uint16_t _n_electrodes = 6;

    int _pin1, _pin2;

    bool saveData = true;
    bool protoCyb = false;

    std::ofstream _file;
    bool _need_to_write_header;
    std::string _filename;
    clock::time_point _start_time;
    int _cnt;

    // Myo thresholds for state machine
    Param<int> _forward_upper;
    Param<int> _forward_lower;
    Param<int> _backward_upper;
    Param<int> _backward_lower;
};

#endif // MYO_2ELECTRODES_H
