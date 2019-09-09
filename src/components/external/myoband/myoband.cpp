#include "myoband.h"
#include "utils/log/log.h"

Myoband::Myoband()
    : ThreadedLoop("myoband", 0.0025)
    , _serial("/dev/myoband", 115200)
    , _client(nullptr)
    , _acc(Eigen::Vector3f::Zero())
    , _gyro(Eigen::Vector3f::Zero())
{
    _wd.set_timeout(std::chrono::seconds(10));
    _wd.set_callback([this] { critical() << "Myoband thread timed out"; if(_thread.joinable()) _thread.detach(); });

    _menu->set_description("Myoband");
    _menu->set_code("mb");
}

Myoband::~Myoband()
{
    stop_and_join();
}

bool Myoband::setup()
{
    info() << "Myoband publishes to " << full_name() << "/acc & " << full_name() << "/emg_rms";
    auto emg_callback = [this](myolinux::myo::EmgSample sample) {
        static const int window_size = 20;
        static Eigen::MatrixXd emgs_history(window_size, sample.size());
        static int history_idx = 0;
        std::string payload;

        std::lock_guard lock(_mutex);

        if (_emgs.size() < static_cast<int>(sample.size()))
            _emgs.resize(sample.size());
        if (_emgs_rms.size() < static_cast<int>(sample.size()))
            _emgs_rms.resize(sample.size());

        for (unsigned int i = 0; i < sample.size(); i++) {
            _emgs[i] = sample[i];
            emgs_history(history_idx++, static_cast<Eigen::Index>(i)) = sample[i];
            _emgs_rms[i] = static_cast<int32_t>(std::round(sqrt(emgs_history.col(static_cast<Eigen::Index>(i)).squaredNorm() / window_size)));
            if (history_idx >= window_size) {
                history_idx = 0;
            }

            payload += std::to_string(_emgs_rms[i]) + " ";
        }
        _mqtt.publish(full_name() + "/emg_rms", payload);
    };

    auto imu_callback = [this](myolinux::myo::OrientationSample ori, myolinux::myo::AccelerometerSample acc, myolinux::myo::GyroscopeSample gyr) {
        std::string payload;
        std::lock_guard lock(_mutex);

        _imu = Eigen::Quaternionf(ori[0] / myolinux::myo::OrientationScale,
            ori[1] / myolinux::myo::OrientationScale,
            ori[2] / myolinux::myo::OrientationScale,
            ori[3] / myolinux::myo::OrientationScale);

        for (unsigned int i = 0; i < 3; i++) {
            _acc[static_cast<Eigen::Index>(i)] = acc[i] / myolinux::myo::AccelerometerScale;
            _gyro[static_cast<Eigen::Index>(i)] = gyr[i] / myolinux::myo::GyroscopeScale;
            payload += std::to_string(acc[i]) + " ";
        }
        _mqtt.publish(full_name() + "/acc", payload);
    };

    _wd.ping();

    _client = nullptr;
    info("MYOBAND : Trying to connect... Try to plug in/unplug the USB port");
    _client = new myolinux::myo::Client(_serial);
    _client->connect();
    _client->setSleepMode(myolinux::myo::SleepMode::NeverSleep);
    _client->setMode(myolinux::myo::EmgMode::SendEmg, myolinux::myo::ImuMode::SendData, myolinux::myo::ClassifierMode::Disabled);
    _client->onEmg(emg_callback);
    _client->onImu(imu_callback);

    return true;
}

void Myoband::loop(double, clock::time_point)
{
    static bool connected = false;

    if (!connected && _client->connected()) {
        info("MYOBAND : Connected");
        connected = true;
    }

    try {
        _wd.ping();
        _client->listen();
    } catch (myolinux::myo::DisconnectedException& e) {
        critical() << e.what();
        connected = false;
        delete _client;
        setup();
    }
}

void Myoband::cleanup()
{
    if (connected()) {
        _client->disconnect();
    }

    _wd.stop_and_join();

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

std::vector<int8_t> Myoband::get_emgs()
{
    std::lock_guard lock(_mutex);
    return _emgs;
}

std::vector<int32_t> Myoband::get_emgs_rms()
{
    std::lock_guard lock(_mutex);
    return _emgs_rms;
}

Eigen::Quaternionf Myoband::get_imu()
{
    std::lock_guard lock(_mutex);
    return _imu;
}

Eigen::Vector3f Myoband::get_acc()
{
    std::lock_guard lock(_mutex);
    return _acc;
}

Eigen::Vector3f Myoband::get_gyro()
{
    std::lock_guard lock(_mutex);
    return _gyro;
}
