#ifndef GENERAL_FORMULATION_H
#define GENERAL_FORMULATION_H

#include "algo/lawjacobian.h"
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
    int _Lt;
    double _Lua;
    double _Lfa;
    double _lhand;
    double l[nbLinks];
    int _lambdaW, _lambda;
    double theta[nbLinks];
    double _threshold[nbLinks];
};

#endif // GENERAL_FORMULATION_H
