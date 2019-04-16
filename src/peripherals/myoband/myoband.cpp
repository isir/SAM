#include "myoband.h"
#include "utils/mqttclient.h"
#include <QDebug>
#include <QMutexLocker>

Myoband::Myoband()
    : BasicController(0.0025)
    , _client(nullptr)
    , _acc(Eigen::Vector3d::Zero())
    , _gyro(Eigen::Vector3d::Zero())
{
    _menu.set_title("Myoband");
    _menu.set_code("mb");

    QObject::connect(&_mqtt_timer, &QTimer::timeout, this, &Myoband::mqtt_timer_callback);
    _mqtt_timer.start(30);
}

Myoband::~Myoband()
{
}

bool Myoband::setup()
{
    _client = nullptr;
    try {
        qInfo("MYOBAND : Trying to connect... Try to plug in/unplug the USB port");
        _client = new myolinux::myo::Client(myolinux::Serial("/dev/ttyACM0", 115200));
        _client->connect();
        _client->setSleepMode(myolinux::myo::SleepMode::NeverSleep);
        _client->setMode(myolinux::myo::EmgMode::SendEmg, myolinux::myo::ImuMode::SendData, myolinux::myo::ClassifierMode::Disabled);
        _client->onEmg([this](myolinux::myo::EmgSample sample) {
            static const int window_size = 20;
            static Eigen::MatrixXd emgs_history(window_size, sample.size());
            static unsigned int history_idx = 0;

            QMutexLocker lock(&_mutex);

            if (_emgs.size() < static_cast<int>(sample.size()))
                _emgs.resize(sample.size());
            if (_emgs_rms.size() < static_cast<int>(sample.size()))
                _emgs_rms.resize(sample.size());

            for (unsigned i = 0; i < sample.size(); i++) {
                _emgs[i] = sample[i];
                emgs_history(history_idx++, i) = sample[i];
                _emgs_rms[i] = sqrt(emgs_history.col(i).squaredNorm() / window_size);
                if (history_idx >= window_size) {
                    history_idx = 0;
                }
            }
        });
        _client->onImu([this](myolinux::myo::OrientationSample ori, myolinux::myo::AccelerometerSample acc, myolinux::myo::GyroscopeSample gyr) {
            QMutexLocker lock(&_mutex);
            _imu = Eigen::Quaterniond(ori[0] / myolinux::myo::OrientationScale,
                ori[1] / myolinux::myo::OrientationScale,
                ori[2] / myolinux::myo::OrientationScale,
                ori[3] / myolinux::myo::OrientationScale);

            for (unsigned i = 0; i < 3; i++) {
                _acc[i] = acc[i] / myolinux::myo::AccelerometerScale;
            }
            for (unsigned i = 0; i < 3; i++) {
                _gyro[i] = gyr[i] / myolinux::myo::GyroscopeScale;
            }
        });
    } catch (std::exception& e) {
        qCritical() << e.what();
        _client = nullptr;
        return false;
    }
    return true;
}

void Myoband::loop(double, double)
{
    static bool connected = false;
    if (!connected && _client->connected()) {
        qInfo("MYOBAND : Connected");
        connected = true;
    }
    try {
        _client->listen();
    } catch (std::exception& e) {
        qCritical() << e.what();
        _client->disconnect();
        connected = false;
        setup();
    }
}

void Myoband::cleanup()
{
    if (_client->connected()) {
        _client->disconnect();
    }
    delete _client;
}

bool Myoband::connected()
{
    if (_client) {
        return _client->connected();
    } else {
        return false;
    }
}

void Myoband::mqtt_timer_callback()
{
    MqttClient& mqtt = MqttClient::instance();

    if (connected() && mqtt.state() == QMqttClient::Connected) {
        QByteArray mqtt_payload;
        Eigen::Vector3d acc = get_acc();

        for (int i = 0; i < 3; ++i) {
            mqtt_payload.append(QByteArray::number(acc[i]) + " ");
        }
        mqtt_payload.chop(1);
        mqtt.publish(QString("sam/myoband/acc"), mqtt_payload);
    }
}
