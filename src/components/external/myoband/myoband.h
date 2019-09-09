#ifndef MYOBAND_H
#define MYOBAND_H

#include "myoLinux/myoclient.h"
#include "myoLinux/serial.h"
#include "utils/interfaces/mqtt_user.h"
#include "utils/watchdog.h"
#include <utils/threaded_loop.h>
#include <eigen3/Eigen/Dense>
#include <vector>

class Myoband : public ThreadedLoop, public MqttUser {
public:
    Myoband();
    ~Myoband() override;

    bool connected();

    std::vector<int8_t> get_emgs();
    std::vector<int32_t> get_emgs_rms();
    Eigen::Quaternionf get_imu();
    Eigen::Vector3f get_acc();
    Eigen::Vector3f get_gyro();

private:
    bool setup() override;
    void loop(double dt, clock::time_point time) override;
    void cleanup() override;

    myolinux::Serial _serial;
    myolinux::myo::Client* _client;

    Watchdog _wd;

    std::vector<int8_t> _emgs;
    std::vector<int32_t> _emgs_rms;
    Eigen::Matrix<float, 3, 1, Eigen::DontAlign> _acc;
    Eigen::Matrix<float, 3, 1, Eigen::DontAlign> _gyro;
    Eigen::Quaternion<float, Eigen::DontAlign> _imu;
};

#endif // MYOBAND_H
