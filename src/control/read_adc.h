#ifndef READADC_H
#define READADC_H

#include "sam/sam.h"
#include "utils/interfaces/mqtt_user.h"
#include "utils/threaded_loop.h"
#include <fstream>

class ReadADC : public ThreadedLoop, public MqttUser {
public:
    ReadADC(std::shared_ptr<SAM::Components> robot);
    ~ReadADC() override;

    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;
    clock::time_point _start_time;

private:
    void readAllADC();

    std::shared_ptr<SAM::Components> _robot;
    static const uint16_t _n_electrodes = 6;
    uint16_t _electrodes[_n_electrodes];

    // indicate whether to save data
    bool saveData = true;
    std::ofstream _file;
};

#endif // READADC_H
