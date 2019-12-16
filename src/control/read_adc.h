#ifndef READADC_H
#define READADC_H

#include "sam/sam.h"
#include "utils/threaded_loop.h"
#include "utils/interfaces/mqtt_user.h"

class ReadADC : public ThreadedLoop, public MqttUser {
public:
    ReadADC(std::shared_ptr<SAM::Components> robot);
    ~ReadADC() override;

    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;

private:
    std::shared_ptr<SAM::Components> _robot;
    int _th_low;
    int _th_high;
};

#endif // READADC_H
