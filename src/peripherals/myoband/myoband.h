#ifndef MYOBAND_H
#define MYOBAND_H

#include "myoLinux/myoclient.h"
#include "myoLinux/serial.h"
#include <QMutex>
#include <QVector>
#include <control/basiccontroller.h>
#include <eigen3/Eigen/Dense>

class Myoband : public BasicController {

public:
    Myoband();
    ~Myoband();

    bool setup();
    void loop(double dt, double time);
    void cleanup();

    bool connected() { return _client.connected(); }

    QVector<qint8> get_emgs()
    {
        QMutexLocker lock(&_mutex);
        return _emgs;
    }
    QVector<qint32> get_emgs_rms()
    {
        QMutexLocker lock(&_mutex);
        return _emgs_rms;
    }
    Eigen::Quaterniond get_imu()
    {
        QMutexLocker lock(&_mutex);
        return _imu;
    }
    Eigen::Vector3d get_acc()
    {
        QMutexLocker lock(&_mutex);
        return _acc;
    }
    Eigen::Vector3d get_gyro()
    {
        QMutexLocker lock(&_mutex);
        return _gyro;
    }

private:
    myolinux::myo::Client _client;
    QMutex _mutex;

    QVector<qint8> _emgs;
    QVector<qint32> _emgs_rms;
    Eigen::Vector3d _acc;
    Eigen::Vector3d _gyro;
    Eigen::Quaterniond _imu;
};

#endif // MYOBAND_H
