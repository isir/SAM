#ifndef SAMANAGER_H
#define SAMANAGER_H

#include "control/compensation_imu.h"
#include "control/compensation_optitrack.h"
#include "control/demo.h"
#include "control/demo_imu.h"
#include "control/jf_imu.h"
#include "control/jf_opti.h"
#include "control/matlab_receiver.h"
#include "control/remote_computer_control.h"
#include "control/voluntary_control.h"
#include "sam/sam.h"
#include "sam/system_monitor.h"
#include "ui/menu/menu_console.h"
#include "ui/menu/menu_mqtt.h"
#include "utils/interfaces/mqtt_user.h"
#include "ux/menu/menu_backend.h"
#include <condition_variable>

class SAManager : public MqttUser {
public:
    explicit SAManager();
    ~SAManager();

    void run();

private:
    void fill_menus();
    void instantiate_controllers();
    void autostart_demo();

    std::condition_variable _cv;
    std::mutex _cv_mutex;

    std::unique_ptr<VoluntaryControl> _vc;
    std::unique_ptr<JacobianFormulationOpti> _jfOpti;
    std::unique_ptr<JacobianFormulationIMU> _jfIMU;
    std::unique_ptr<CompensationOptitrack> _opti;
    std::unique_ptr<RemoteComputerControl> _rm;
    std::unique_ptr<MatlabReceiver> _mr;
    std::unique_ptr<Demo> _demo;
    std::unique_ptr<DemoIMU> _demoimu;
    std::unique_ptr<SystemMonitor> _sm;

    std::unique_ptr<MenuBackend> _main_menu;
    std::unique_ptr<MenuMQTT> _menu_mqtt_binding;
    std::unique_ptr<MenuConsole> _menu_console_binding;

    std::shared_ptr<SAM::Components> _robot;
};

#endif // SAMANAGER_H
