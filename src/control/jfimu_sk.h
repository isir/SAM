#ifndef JACOBIAN_FORMULATION_IMU_SK_H
#define JACOBIAN_FORMULATION_IMU_SK_H

#include "algo/lawjacobian.h"
#include "sam/sam.h"
#include "utils/named_object.h"
#include "utils/socket.h"
#include "utils/threaded_loop.h"
#include <fstream>
#include <components/internal/gpio/gpio.h>

class JacobianFormulationIMU_sk : public ThreadedLoop {
public:
    explicit JacobianFormulationIMU_sk(std::string name, std::string filename, std::shared_ptr<SAM::Components> robot);
    ~JacobianFormulationIMU_sk() override;

    std::shared_ptr<SAM::Components> _robot;
    std::ofstream _file;
    bool _need_to_write_header;
    std::string _filename;
    clock::time_point _start_time;

    LawJacobian _lawJ;

    void tare_IMU();
    void analog_IMU();
    void displayPin();
    void calibrations();
    void listenArduino();

    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;

protected:
    virtual void initializationLaw(Eigen::Quaterniond qHi, double p);
    virtual void initialPositionsLaw(Eigen::Quaterniond qHa, Eigen::Quaterniond qHi, Eigen::Quaterniond qT, Eigen::Quaterniond qA, double theta[], int lt, int lsh, int l[], int nbDOF, int cnt, int init_cnt);
    virtual void controlLaw(Eigen::Quaterniond qHa, Eigen::Quaterniond qHi, Eigen::Quaterniond qT, Eigen::Quaterniond qA, double theta[], int lt, int lsh, int l[], int nbDOF, int k, double lambda[], double threshold[], int cnt, int init_cnt);

private:
    Param<int> _k; // damping parameter for inverse kinematics
    Param<int> _lt; // length of the trunk
    Param<int> _lsh; // length between neck and shoulder
    Param<int> _lua; // upper-arm length
    Param<int> _lfa; // forearm length
    Param<int> _lwrist; // length between flexion and pronosupination joints
    Param<double> _lambdaE; // gain for elbow flexion
    Param<double> _lambdaWF; // gain for wrist flexion
    Param<double> _lambdaWPS; // gain for wrist pronosupination
    Param<double> _thresholdE; // threshold for elbow flexion
    Param<double> _thresholdWF; // threshold for wrist flexion
    Param<double> _thresholdWPS; // thresold for wrist pronosupination

    int _nbDOF;
    int _init_cnt = 10;
    int _cnt;
    double _lambda[nbLinks];
    GPIO _pin_up;
    GPIO _pin_down;
    double _theta[nbLinks];
    double _threshold[nbLinks];
    int _l[nbLinks];

    Socket _receiverArduino;
    // GPIO _pinArduino;
    unsigned int _infoSent;

    uint16_t _emg[2];
    static const uint16_t _n_electrodes = 6;
    int _th_low[_n_electrodes];
    int _th_high[_n_electrodes];
    std::ifstream _param_file;

    Eigen::Quaterniond _qHip, _qTrunk, _qHand, _qArm;

    // boolean to indicate which prototype and whether we save data
    bool protoCyb = false;
    bool saveData = true;
};

#endif // JACOBIAN_FORMULATION_IMU_SK_H
