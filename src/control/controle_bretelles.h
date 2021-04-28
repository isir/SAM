#ifndef CONTROLEBRETELLES_H
#define CONTROLEBRETELLES_H

#include "sam/sam.h"
#include "utils/socket.h"
#include "utils/threaded_loop.h"
#include <fstream>

class ControleBretelles : public ThreadedLoop
{
public:
    explicit ControleBretelles(std::shared_ptr<SAM::Components> robot);
    ~ControleBretelles() override;

private:
    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;
    void unpack_data(std::vector<std::byte>);
    void demo_mode();
    void controle_vitesse_p();
    void controle_vitesse_pi();
    void controle_2DOF();
    void calib_bretelles();

    std::shared_ptr<SAM::Components> _robot;
    Socket _socket;

    int _avd;
    int _avg;
    int _ard;
    int _arg;
    int _cg;
    int _cd;
    int _label;
    int _button;
    double _gain;

    int _mode = 0;
    int _val_init[6] = {0};
    int _start_button;
    bool _start;
    int _cnt;

    std::ofstream _file;
};

#endif // CONTROLEBRETELLES_H
