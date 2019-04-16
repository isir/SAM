#ifndef MYOBAND_H
#define MYOBAND_H

#include "myoLinux/myoclient.h"
#include "myoLinux/serial.h"
#include <QMutex>
#include <QTimer>
#include <QVector>
#include <control/basiccontroller.h>
#include <eigen3/Eigen/Dense>

class Myoband : public BasicController {
    Q_OBJECT
public:
    Myoband();
    ~Myoband();

    bool setup();
    void loop(double dt, double time);
    void cleanup();

    bool connected();

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
    myolinux::myo::Client* _client;
    QMutex _mutex;
    QTimer _mqtt_timer;

    QVector<qint8> _emgs;
    QVector<qint32> _emgs_rms;
    Eigen::Matrix<double, 3, 1, Eigen::DontAlign> _acc;
    Eigen::Matrix<double, 3, 1, Eigen::DontAlign> _gyro;
    Eigen::Quaternion<double, Eigen::DontAlign> _imu;

private slots:
    void mqtt_timer_callback();
};

#endif // MYOBAND_H
