#include "samanager.h"
#include <wiringPi.h>

std::shared_ptr<QMqttClient> SAManager::_mqtt(std::make_shared<QMqttClient>());
std::shared_ptr<Logger> SAManager::_log(std::make_shared<Logger>(SAManager::_mqtt));

static void message_handler(QtMsgType type, const QMessageLogContext& context, const QString& msg)
{
    SAManager::log()->async_handle_message(type, context, msg);
}

SAManager::SAManager(QCoreApplication* a, QObject* parent)
    : QObject(parent)
{
    _main_menu = std::make_shared<ConsoleMenu>(_mqtt, "Main menu", "main");
    _buzzer_submenu = std::make_shared<ConsoleMenu>(_mqtt, "Buzzer submenu", "buzzer");

    QObject::connect(_main_menu.get(), &ConsoleMenu::finished, a, &QCoreApplication::quit, Qt::QueuedConnection);
    QObject::connect(_mqtt.get(), &QMqttClient::connected, this, &SAManager::mqtt_connected_callback);

    _settings.beginGroup("MQTT");
    _mqtt->setHostname(_settings.value("hostname", "192.168.0.130").toString());
    _mqtt->setPort(_settings.value("port", 1883).toInt());
    _mqtt->connectToHost();
    _settings.endGroup();
}

SAManager::~SAManager()
{
    _robot.leds->set(LedStrip::none, 10);
}

void SAManager::mqtt_connected_callback()
{
    qDebug() << "Connected to broker";
    qInstallMessageHandler(&message_handler);

    wiringPiSetup();

    _settings.beginGroup("Buzzer");
    _robot.buzzer = std::make_shared<Buzzer>(_settings.value("pin", 29).toInt());
    _settings.endGroup();

    _robot.leds = std::make_shared<LedStrip>();

    try {
        _robot.wrist = std::make_shared<PronoSupination>(_mqtt);
    } catch (std::exception& e) {
        qCritical() << "Couldn't access the wrist -" << e.what();
    }

    try {
        _robot.elbow = std::make_shared<OsmerElbow>(_mqtt);
    } catch (std::exception& e) {
        qCritical() << "Couldn't access the elbow -" << e.what();
    }

    try {
        _robot.hand = std::make_shared<TouchBionicsHand>(_mqtt);
    } catch (std::exception& e) {
        qCritical() << "Couldn't access the hand -" << e.what();
    }

    try {
        _robot.myoband = std::make_shared<Myoband>(_mqtt);
        _robot.myoband->start();
    } catch (std::exception& e) {
        qCritical() << "Couldn't access the Myoband dongle -" << e.what();
    }

    try {
        _robot.arm_imu = std::make_shared<XIMU>("/dev/ximu_red", XIMU::XIMU_LOGLEVEL_NONE, 115200);
    } catch (std::exception& e) {
        qCritical() << "Couldn't access the red IMU -" << e.what();
    }

    try {
        _robot.trunk_imu = std::make_shared<XIMU>("/dev/ximu_white", XIMU::XIMU_LOGLEVEL_NONE, 115200);
    } catch (std::exception& e) {
        qCritical() << "Couldn't access the white IMU -" << e.what();
    }

    _robot.adc = std::make_shared<Adafruit_ADS1115>("/dev/i2c-1", 0x48);

    _settings.beginGroup("Optitrack");
    _robot.optitrack = std::make_shared<OptiListener>();
    _robot.optitrack->begin(_settings.value("port", 1511).toInt());
    _settings.endGroup();

    _buzzer_submenu->addItem(ConsoleMenuItem("Single Buzz", "sb", [this](QString) { _robot.buzzer->makeNoise(BuzzerConfig::STANDARD_BUZZ); }));
    _buzzer_submenu->addItem(ConsoleMenuItem("Double Buzz", "db", [this](QString) { _robot.buzzer->makeNoise(BuzzerConfig::DOUBLE_BUZZ); }));
    _buzzer_submenu->addItem(ConsoleMenuItem("Triple Buzz", "tb", [this](QString) { _robot.buzzer->makeNoise(BuzzerConfig::TRIPLE_BUZZ); }));
    _main_menu->addItem(*_buzzer_submenu);

    if (_robot.elbow && _robot.wrist) {
        _vc = std::make_shared<VoluntaryControl>(_robot, _mqtt);
        _main_menu->addItem(_vc->menu());
        if (_robot.hand) {
            _rm = std::make_shared<RemoteComputerControl>(_robot, _mqtt);
            _main_menu->addItem(_rm->menu());
            _mr = std::make_shared<MatlabReceiver>(_robot, _mqtt);
            _main_menu->addItem(_mr->menu());
            if (_robot.arm_imu && _robot.trunk_imu) {
                _opti = std::make_shared<CompensationOptitrack>(_robot, _mqtt);
                _main_menu->addItem(_opti->menu());
            }
            if (_robot.myoband) {
                _demo = std::make_shared<Demo>(_robot, _mqtt);
            }
        }
    }

    _main_menu->activate();

    if (_demo) {
        _main_menu->addItem(_demo->menu());
        _demo->menu().activate();
        _demo->start();
    }

    _sm = std::make_shared<SystemMonitor>(_mqtt);
    _sm->start();

    _robot.leds->set(LedStrip::white, 10);
}
