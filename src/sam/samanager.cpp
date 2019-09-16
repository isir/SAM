#include "samanager.h"
#include "utils/log/log.h"
#include <unistd.h>
#include <wiringPi.h>

SAManager::SAManager()
    : _main_menu(std::make_unique<MenuBackend>("main", "Main menu", [this] { _cv.notify_one(); }))
{
    wiringPiSetup();

    pinMode(SAM::Components::pin_demo, INPUT);
    pullUpDnControl(SAM::Components::pin_demo, PUD_UP);

    if (isatty(fileno(stdin))) {
        _menu_console_binding = std::make_unique<MenuConsole>();
    } else {
        info() << "No TTY detected, console UI disabled.";
    }
    _menu_mqtt_binding = std::make_unique<MenuMQTT>();

    _sm = std::make_unique<SystemMonitor>();
    _sm->start();
}

SAManager::~SAManager()
{
    _menu_mqtt_binding->show_message("Exited gracefully.");
    if (_robot->user_feedback.leds)
        _robot->user_feedback.leds->set(LedStrip::none, 10);
}

void SAManager::run()
{
    _robot = std::make_shared<SAM::Components>();
    _robot->user_feedback.leds->set(LedStrip::white, 10);

    instantiate_controllers();
    fill_menus();
    autostart_demo();

    std::unique_lock lock(_cv_mutex);
    _cv.wait(lock);
}

void SAManager::fill_menus()
{
    std::shared_ptr<MenuBackend> buzzer_submenu = std::make_shared<MenuBackend>("buzzer", "Buzzer submenu");
    buzzer_submenu->add_item("sb", "Single Buzz", [this](std::string) { _robot->user_feedback.buzzer->makeNoise(Buzzer::STANDARD_BUZZ); });
    buzzer_submenu->add_item("db", "Double Buzz", [this](std::string) { _robot->user_feedback.buzzer->makeNoise(Buzzer::DOUBLE_BUZZ); });
    buzzer_submenu->add_item("tb", "Triple Buzz", [this](std::string) { _robot->user_feedback.buzzer->makeNoise(Buzzer::TRIPLE_BUZZ); });
    _main_menu->add_item(buzzer_submenu);

    _main_menu->add_submenu_from_user(_robot->joints.wrist_flexion);
    _main_menu->add_submenu_from_user(_robot->joints.shoulder_medial_rotation);
    _main_menu->add_submenu_from_user(_robot->joints.wrist_pronation);
    _main_menu->add_submenu_from_user(_robot->joints.elbow_flexion);
    _main_menu->add_submenu_from_user(_robot->joints.hand);
    _main_menu->add_submenu_from_user(_vc);
    _main_menu->add_submenu_from_user(_rm);
    _main_menu->add_submenu_from_user(_mr);
    _main_menu->add_submenu_from_user(_opti);
    _main_menu->add_submenu_from_user(_demo);
    _main_menu->add_submenu_from_user(_demoimu);
    _main_menu->add_submenu_from_user(_galf);

    _main_menu->activate();
}

void SAManager::instantiate_controllers()
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
    try {
        _demoimu = std::make_unique<DemoIMU>(_robot);
    } catch (std::exception&) {
    }
    try {
        _galf = std::make_unique<GeneralFormulation>(_robot);
    } catch (std::exception&) {
    }
}

void SAManager::autostart_demo()
{
    if (_demo) {
        if (digitalRead(SAM::Components::pin_demo)) {
            _robot->user_feedback.buzzer->makeNoise(Buzzer::SHORT_BUZZ);
        } else {
            _robot->user_feedback.buzzer->makeNoise(Buzzer::DOUBLE_BUZZ);
            _main_menu->activate_item("demo");
            _demo->start();
        }
    }
}
