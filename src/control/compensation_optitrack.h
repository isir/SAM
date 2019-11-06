#ifndef COMPENSATION_OPTITRACK_H
#define COMPENSATION_OPTITRACK_H

#include "algo/lawopti.h"
#include "components/external/optitrack/optitrack_listener.h"
#include "sam/sam.h"
#include "utils/socket.h"
#include "utils/threaded_loop.h"
#include <fstream>

class CompensationOptitrack : public ThreadedLoop {
public:
    explicit CompensationOptitrack(std::shared_ptr<SAM::Components> robot);
    ~CompensationOptitrack() override;

    void start(std::string filename = std::string());
    void stop();
    void zero();
    void tareIMU();
    void display_parameters();
    void display_lengths();

private:
    enum Mode {
        COMP,
        VOL
    };

    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;

    void on_activated();
    void on_new_data_compensation(optitrack_data_t data, double dt, clock::time_point time);
    void on_new_data_vol(optitrack_data_t data, double dt, clock::time_point time);
    void read_optiData(optitrack_data_t data);
    void on_def();
    void listenArduino();

    std::shared_ptr<SAM::Components> _robot;
    Socket _receiver;
    Socket _receiverArduino;

    int _previous_elapsed;
    double _old_time;

    Mode _mode;

    clock::time_point _abs_time_start;
    clock::time_point _time_start;

    std::ofstream _file;
    bool _need_to_write_header;
    LawOpti _lawopti;
    unsigned int _cnt;
    unsigned int _ind;
    unsigned int _infoSent;

    int _Lt;
    double _Lua;
    double _Lfa;
    double _l;
    int _lsh;
    int _lambdaW, _lambda;
    double _thresholdW, _threshold;
    int _pinArduino;
};

#endif // COMPENSATION_OPTITRACK_H
