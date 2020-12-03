#ifndef JACOBIAN_FORMULATION_OPTI_H
#define JACOBIAN_FORMULATION_OPTI_H

#include "algo/lawjacobian.h"
#include "sam/sam.h"
#include "utils/socket.h"
#include "utils/threaded_loop.h"
#include "utils/interfaces/mqtt_user.h"
#include <fstream>

class JacobianFormulationOpti : public ThreadedLoop, public MqttUser {
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
    void receiveData();
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
    //    int _Lua;
    //    int _Lfa;
    //    int _lhand;
    //    int _lwrist;
    double _lambda[nbLinks];
    int _pin_up;
    int _pin_down;
    int _electrodes[2];
    double theta[nbLinks];
    double _threshold[nbLinks];
    int l[nbLinks];
    int nbRigidBodies;

    Param<int> _k;
    Param<double> _lua;
    Param<double> _lfa;
    Param<double> _lwrist;
    Param<double> _lambdaE;
    Param<double> _lambdaWF;
    Param<double> _lambdaWPS;
    Param<double> _thresholdE;
    Param<double> _thresholdWF;
    Param<double> _thresholdWPS;

    // boolean to indicate which prototype and whether we save data
    bool protoCyb = true;
    bool saveData = false;
};

#endif // GENERAL_FORMULATION_H
