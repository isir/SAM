#ifndef CONTROLEBRETELLES_H
#define CONTROLEBRETELLES_H

#include "sam/sam.h"
#include "utils/socket.h"
#include "utils/threaded_loop.h"

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

    int _avd;
    int _avg;
    int _ard;
    int _arg;
    int _avc;
    int _arc;
    int _label;
    double _gain;

    Socket _socket;
    std::shared_ptr<SAM::Components> _robot;
};

#endif // CONTROLEBRETELLES_H
