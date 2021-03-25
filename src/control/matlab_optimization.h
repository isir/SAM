#ifndef MATLAB_OPTIMIZATION_H
#define MATLAB_OPTIMIZATION_H

#include "sam/sam.h"
#include "utils/socket.h"
#include "utils/threaded_loop.h"
#include <fstream>

class MatlabOptimization : public ThreadedLoop {
public:
    explicit MatlabOptimization(std::shared_ptr<SAM::Components> robot);
    ~MatlabOptimization() override;

private:
    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;
    void calibrations();
    void set_velocity_motors(double speed1, double speed2);

    Socket _socket;
    std::ofstream _file;
    bool _need_to_write_header;
    int _cnt;
    clock::time_point _start_time;

    std::shared_ptr<SAM::Components> _robot;

    int nbRigidBodies;
    double _theta[2], _thetaDot[2], _lambda[2];
    double _qd;
    bool saveData = false;
    bool protoCyb = false;
};

#endif // MATLAB_OPTIMIZATION_H
