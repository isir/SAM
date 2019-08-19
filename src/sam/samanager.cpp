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

    _main_menu->add_item(_robot->joints.wrist_flexion->menu());
    _main_menu->add_item(_robot->joints.shoulder_medial_rotation->menu());
    _main_menu->add_item(_robot->joints.wrist_pronation->menu());
    _main_menu->add_item(_robot->joints.elbow_flexion->menu());
    _main_menu->add_item(_robot->joints.hand->menu());
    _main_menu->add_item(_vc->menu());
    _main_menu->add_item(_rm->menu());
    _main_menu->add_item(_mr->menu());
    _main_menu->add_item(_opti->menu());
    _main_menu->add_item(_demo->menu());

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
