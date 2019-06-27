#include "samanager.h"
#include <unistd.h>
#include <wiringPi.h>

#include "peripherals/actuators/custom_elbow.h"
#include "peripherals/actuators/osmer_elbow.h"

#include "peripherals/actuators/pronosupination.h"
#include "peripherals/actuators/shoulder_rotator.h"
#include "peripherals/actuators/wrist_rotator.h"

std::shared_ptr<Logger> SAManager::_log;

static void message_handler(QtMsgType type, const QMessageLogContext& context, const QString& msg)
{
    SAManager::log()->async_handle_message(type, context, msg);
}

SAManager::SAManager(QCoreApplication* a, QObject* parent)
    : QObject(parent)
    , MqttUser("SAManager", AUTOCONNECT)
    , _main_menu(std::make_unique<MenuBackend>("main", "Main menu"))
{

    QObject::connect(_main_menu.get(), &MenuBackend::finished, a, &QCoreApplication::quit, Qt::QueuedConnection);
    QObject::connect(&_mqtt, &QMqttClient::connected, this, &SAManager::mqtt_connected_callback);

    wiringPiSetup();

    pinMode(SAM::Components::pin_demo, INPUT);
    pullUpDnControl(SAM::Components::pin_demo, PUD_UP);

    if (isatty(fileno(stdin))) {
        _menu_console_binding = std::make_unique<MenuConsole>();
    } else {
        qInfo() << "No TTY detected, console UI disabled.";
    }
    _menu_mqtt_binding = std::make_unique<MenuMQTT>();
}

SAManager::~SAManager()
{
    _menu_mqtt_binding->show_message("Exited gracefully.");
    if (_robot->user_feedback.leds)
        _robot->user_feedback.leds->set(LedStrip::none, 10);
}

void SAManager::internal_init()
{
    _robot = std::make_shared<SAM::Components>();
    _robot->user_feedback.leds->set(LedStrip::blue, 10);

    instanciate_controllers();
    fill_menus();
    autostart_demo();
}

void SAManager::fill_menus()
{
    std::shared_ptr<MenuBackend> buzzer_submenu = std::make_shared<MenuBackend>("buzzer", "Buzzer submenu");
    buzzer_submenu->add_item("sb", "Single Buzz", [this](QString) { _robot->user_feedback.buzzer->makeNoise(BuzzerConfig::STANDARD_BUZZ); });
    buzzer_submenu->add_item("db", "Double Buzz", [this](QString) { _robot->user_feedback.buzzer->makeNoise(BuzzerConfig::DOUBLE_BUZZ); });
    buzzer_submenu->add_item("tb", "Triple Buzz", [this](QString) { _robot->user_feedback.buzzer->makeNoise(BuzzerConfig::TRIPLE_BUZZ); });
    _main_menu->add_item(buzzer_submenu);

    _main_menu->add_submenu_from_user(_robot->joints.wrist_flex);
    _main_menu->add_submenu_from_user(_robot->joints.shoulder);
    _main_menu->add_submenu_from_user(_robot->joints.wrist_pronosup);
    _main_menu->add_submenu_from_user(_robot->joints.elbow);
    _main_menu->add_submenu_from_user(_robot->joints.hand);
    _main_menu->add_submenu_from_user(_vc);
    _main_menu->add_submenu_from_user(_rm);
    _main_menu->add_submenu_from_user(_mr);
    _main_menu->add_submenu_from_user(_opti);
    _main_menu->add_submenu_from_user(_demo);

    _main_menu->activate();
}

void SAManager::instanciate_controllers()
{
    try {
        _vc = std::make_unique<VoluntaryControl>(_robot);
    } catch (std::exception&) {
    }
    try {
        _rm = std::make_unique<RemoteComputerControl>(_robot);
    } catch (std::exception&) {
    }
    try {
        _mr = std::make_unique<MatlabReceiver>(_robot);
    } catch (std::exception&) {
    }
    try {
        _opti = std::make_unique<CompensationOptitrack>(_robot);
    } catch (std::exception&) {
    }
    try {
        _demo = std::make_unique<Demo>(_robot);
    } catch (std::exception&) {
    }
}

void SAManager::autostart_demo()
{
    if (_demo) {
        if (digitalRead(SAM::Components::pin_demo)) {
            _robot->user_feedback.buzzer->makeNoise(BuzzerConfig::SHORT_BUZZ);
        } else {
            _robot->user_feedback.buzzer->makeNoise(BuzzerConfig::DOUBLE_BUZZ);
            _demo->menu()->activate();
            _demo->start();
        }
    }
}

void SAManager::mqtt_connected_callback()
{
    qInfo() << "Connected to broker";
    if (!_log) {
        _log = std::make_shared<Logger>();
    }
    qInstallMessageHandler(&message_handler);

    internal_init();

    _sm = std::make_unique<SystemMonitor>();
    _sm->start();

    _robot->user_feedback.leds->set(LedStrip::white, 10);
}
