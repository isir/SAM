#include "myoband.h"
#include <QDebug>
#include <QMutexLocker>

Myoband::Myoband()
try
    : BasicController(0.0025),
      _client(myolinux::Serial("/dev/ttyACM0", 115200)) {
    _client.onEmg([this](myolinux::myo::EmgSample sample) {
        static const int window_size = 20;
        static Eigen::MatrixXd emgs_history(window_size, sample.size());
        static unsigned int history_idx = 0;

        QMutexLocker lock(&_mutex);

        if (_emgs.size() < sample.size())
            _emgs.resize(sample.size());
        if (_emgs_rms.size() < sample.size())
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

    _client.onImu([this](myolinux::myo::OrientationSample ori, myolinux::myo::AccelerometerSample acc, myolinux::myo::GyroscopeSample gyr) {
        QMutexLocker lock(&_mutex);
        for (unsigned i = 0; i < 4; i++) {
            _imu = Eigen::Quaterniond(ori[0] / myolinux::myo::OrientationScale,
                ori[1] / myolinux::myo::OrientationScale,
                ori[2] / myolinux::myo::OrientationScale,
                ori[3] / myolinux::myo::OrientationScale);
        }
        for (unsigned i = 0; i < 3; i++) {
            this->_acc[i] = acc[i] / myolinux::myo::AccelerometerScale;
        }
        for (unsigned i = 0; i < 3; i++) {
            this->_gyro[i] = gyr[i] / myolinux::myo::GyroscopeScale;
        }
    });

    _menu.set_title("Myoband");
    _menu.set_code("mb");
} catch (std::exception& e) {
    qCritical() << e.what();
}

Myoband::~Myoband()
{
}

bool Myoband::setup()
{
    qInfo("MYOBAND : Trying to connect... Try to plug in/unplug the USB port");
    _client.connect();
    _client.setSleepMode(myolinux::myo::SleepMode::NeverSleep);
    _client.setMode(myolinux::myo::EmgMode::SendEmg, myolinux::myo::ImuMode::SendData, myolinux::myo::ClassifierMode::Disabled);
    return true;
}

void Myoband::loop(double, double)
{
    static bool connected = false;
    if (!connected && _client.connected()) {
        qInfo("MYOBAND : Connected");
        connected = true;
    }
    try {
        _client.listen();
    } catch (std::exception& e) {
        qCritical() << e.what();
        _client.disconnect();
        connected = false;
        setup();
    }
}

void Myoband::cleanup()
{
    _client.disconnect();
}
