#ifndef GENERAL_FORMULATION_IMU_H
#define GENERAL_FORMULATION_IMU_H

#include "algo/lawjacobian.h"
#include "sam/sam.h"
#include "utils/socket.h"
#include "utils/threaded_loop.h"
#include <fstream>

class GeneralFormulationIMU : public ThreadedLoop {
public:
    explicit GeneralFormulationIMU(std::shared_ptr<SAM::Components> robot);
    ~GeneralFormulationIMU() override;

private:
    void tare_IMU();
    void displayPin();
    void calibrations();
    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;

    std::shared_ptr<SAM::Components> _robot;
    std::ofstream _file;
    bool _need_to_write_header;
    int _cnt;
    clock::time_point _start_time;

    LawJacobian _lawJ;
    int nbDOF;
    int _lambda[nbLinks];
    int _pin_up;
    int _pin_down;
    double theta[nbLinks];
    double _threshold[nbLinks];
    int l[nbLinks];

    Param<double> _lt; // length of the trunk
    Param<double> _lsh; // length between neck and shoulder
    Param<double> _lua; // upper-arm length
    Param<double> _lfa; // forearm length
    Param<double> _lwrist; // length between flexion and pronosupination joints
    Param<int> _lambdaE; // gain for elbow flexion
    Param<int> _lambdaWF; // gain for wrist flexion
    Param<int> _lambdaWPS; // gain for wrist pronosupination
    Param<double> _thresholdE; // threshold for elbow flexion
    Param<double> _thresholdWF; // threshold for wrist flexion
    Param<double> _thresholdWPS; // thresold for wrist pronosupination
};

#endif // GENERAL_FORMULATION_H
