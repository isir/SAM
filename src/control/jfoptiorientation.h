#ifndef JFOPTIORIENTATION_H
#define JFOPTIORIENTATION_H

#include "algo/lawjacobian.h"
#include "sam/sam.h"
#include "utils/interfaces/mqtt_user.h"
#include "utils/socket.h"
#include "utils/threaded_loop.h"
#include <fstream>

class JFOptiOrientation : public ThreadedLoop, public MqttUser {
public:
    explicit JFOptiOrientation(std::shared_ptr<SAM::Components> robot);
    ~JFOptiOrientation() override;

private:
    void tare_allIMU();
    void tare_whiteIMU();
    void tare_yellowIMU();
    void tare_redIMU();
    void elbowTo90();
    void toPos0();
    void toPos1();
    void toPos2();
    void set_velocity_motors(double speed1, double speed2);
    void displayPin();
    void displayRBnb();
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
    double _k;
    int _lt;
    double _lambda[nbLinks];
    int _pin_up;
    int _pin_down;
    int _electrodes[2];
    double theta[nbLinks];
    double _threshold[nbLinks];
    int l[nbLinks];
    int nbRigidBodies;

    Param<double> _lambdaE; // gain for elbow velocity control
    Param<double> _lambdaWF; // gain for wrist flexion velocity control
    Param<double> _lambdaWPS; // gain for wrist pronosup velocity control
    Param<double> _thresholdE; // threshold for elbow deadzone
    Param<double> _thresholdWF; // threshold for wrist flexion deadzone
    Param<double> _thresholdWPS; // threshold for wrist pronosup deadzone

    // boolean to indicate which prototype and whether we save data
    bool protoCyb = false;
    bool saveData = true;
};

#endif // // JFOPTIORIENTATION_H
