#ifndef TESTIMU_H
#define TESTIMU_H

#include "sam/sam.h"
#include "utils/socket.h"
#include "utils/interfaces/mqtt_user.h"
#include "utils/threaded_loop.h"
#include <fstream>
#include "utils/serial_port.h"

class TestIMU : public ThreadedLoop, public MqttUser  {
public:
    explicit TestIMU(std::shared_ptr<SAM::Components> robot);
    ~TestIMU() override;

private:
    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;

    std::shared_ptr<SAM::Components> _robot;

    void send();

    SerialPort _serial_port;
};

#endif // TESTIMU_H
