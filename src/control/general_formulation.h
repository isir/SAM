#ifndef GENERAL_FORMULATION_H
#define GENERAL_FORMULATION_H

#include "algo/lawjacobian.h"
#include "components/external/optitrack_listener.h"
#include "sam/sam.h"
#include "utils/socket.h"
#include "utils/threaded_loop.h"
#include <fstream>

class GeneralFormulation : public ThreadedLoop {
public:
    explicit GeneralFormulation(std::shared_ptr<SAM::Components> robot);
    ~GeneralFormulation() override;

private:
    void tare_IMU();
    void receiveData();
    void displayPin();
    void calibrations();
    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;

    std::shared_ptr<SAM::Components> _robot;
    Socket _receiver;
    std::ofstream _file;
    bool _need_to_write_header;
    int _cnt;
    clock::time_point _start_time;

    LawJacobian _lawJ;
    int nbDOF;
    int _Lt;
    //    int _Lua;
    //    int _Lfa;
    //    int _lhand;
    //    int _lwrist;
    int _lambda[nbLinks];
    int _pin_up;
    int _pin_down;
    double theta[nbLinks];
    double _threshold[nbLinks];
    int l[nbLinks];
    int nbRigidBodies;

    Param<int> _k;
    Param<double> _lua;
    Param<double> _lfa;
    Param<double> _lwrist;
    Param<int> _lambdaE;
    Param<int> _lambdaWF;
    Param<int> _lambdaWPS;
    Param<double> _thresholdE;
    Param<double> _thresholdWF;
    Param<double> _thresholdWPS;
};

#endif // GENERAL_FORMULATION_H
