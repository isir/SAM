#ifndef PUSHBUTTONS_H
#define PUSHBUTTONS_H

#include "sam/sam.h"
#include "utils/threaded_loop.h"

#include <fstream>

class pushButtons : public ThreadedLoop {
public:
    pushButtons(std::shared_ptr<SAM::Components> robot);
    ~pushButtons() override;

    void tare_IMU();

    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;

private:
    std::shared_ptr<SAM::Components> _robot;

    bool saveData = true;

    std::ofstream _file;
    bool _need_to_write_header;
    std::string _filename;
    clock::time_point _start_time;
    int _cnt;
};

#endif // PUSHBUTTONS_H
