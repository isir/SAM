#ifndef RECORDDATA_H
#define RECORDDATA_H

#include "sam/sam.h"
#include "utils/named_object.h"
#include "utils/socket.h"
#include "utils/threaded_loop.h"
#include <fstream>

class RecordData : public ThreadedLoop {
public:
    explicit RecordData(std::shared_ptr<SAM::Components> robot);
    ~RecordData() override;

    std::shared_ptr<SAM::Components> _robot;
    std::ofstream _file;
    bool _need_to_write_header;
    std::string _filename;
    clock::time_point _start_time;

    void tare_IMU();
    void analog_IMU();

    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;


private:
    Eigen::Quaterniond _qTrunk, _qArm;
};

#endif // RECORDDATA_H
