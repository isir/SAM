#ifndef JACOBIAN_FORMULATION_OPTI_H
#define JACOBIAN_FORMULATION_OPTI_H

#include "algo/lawjacobian.h"
#include "sam/sam.h"
#include "utils/socket.h"
#include "utils/threaded_loop.h"
#include <fstream>

class JacobianFormulationOpti : public ThreadedLoop {
public:
    explicit JacobianFormulationOpti(std::shared_ptr<SAM::Components> robot);
    ~JacobianFormulationOpti() override;

private:
    void tare_allIMU();
    void tare_whiteIMU();
    void tare_yellowIMU();
    void tare_redIMU();
    void elbowTo90();
    void set_velocity_motors(double speed_elbow, double speed_wrist);
    void displayPin();
    void displayRBnb();
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
    int _lt;
    double _lambda[nbLinks];
    int _pin_up;
    int _pin_down;
    int _electrodes[2];
    double theta[nbLinks];
    double _threshold[nbLinks];
    int l[nbLinks];
    int nbRigidBodies;

    Param<int> _k; // gain for damped least square solution
    Param<int> _useIMU; // 0 or 1, indicate if we use IMUs (1) for hip and hand frame or optitrack (0)
    Param<double> _lua; // upperarm length
    Param<double> _lfa; // forearm length
    Param<double> _lwrist; // wrist length (from wrist to hand)
    Param<double> _lambdaE; // gain for elbow velocity control
    Param<double> _lambdaWF; // gain for wrist flexion velocity control
    Param<double> _lambdaWPS; // gain for wrist pronosup velocity control
    Param<double> _thresholdE; // threshold for elbow deadzone
    Param<double> _thresholdWF; // threshold for wrist flexion deadzone
    Param<double> _thresholdWPS; // threshold for wrist pronosup deadzone

    // boolean to indicate which prototype and whether we save data
    bool protoCyb = true;
    bool saveData = false;
};

#endif // GENERAL_FORMULATION_H
