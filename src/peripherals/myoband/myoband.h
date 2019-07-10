#ifndef MYOBAND_H
#define MYOBAND_H

#include "myoLinux/myoclient.h"
#include "myoLinux/serial.h"
#include "ui/mqtt_user.h"
#include <QMqttClient>
#include <QTimer>
#include <QVector>
#include <control/threaded_loop.h>
#include <eigen3/Eigen/Dense>

class Myoband : public ThreadedLoop, public MqttUser {
    Q_OBJECT
public:
    Myoband();
    ~Myoband();

    bool connected();

    QVector<qint8> get_emgs();
    QVector<qint32> get_emgs_rms();
    Eigen::Quaterniond get_imu();
    Eigen::Vector3d get_acc();
    Eigen::Vector3d get_gyro();

private:
    bool setup();
    void loop(double dt, double time);
    void cleanup();

    myolinux::Serial _serial;
    myolinux::myo::Client* _client;
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
