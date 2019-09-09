#ifndef SAMANAGER_H
#define SAMANAGER_H

#include "control/compensation_imu.h"
#include "control/compensation_optitrack.h"
#include "control/demo.h"
#include "control/general_formulation.h"
#include "control/matlab_receiver.h"
#include "control/remote_computer_control.h"
#include "control/voluntary_control.h"
#include "ui/menu_backend.h"
#include "ui/menu_console.h"
#include "ui/menu_mqtt.h"
#include "ui/mqtt_user.h"
#include "utils/logger.h"
#include "utils/sam.h"
#include "utils/settings.h"
#include "utils/system_monitor.h"
#include <QCoreApplication>
#include <QObject>

class SAManager : public QObject, public MqttUser {
    Q_OBJECT
public:
    explicit SAManager(QCoreApplication* a, QObject* parent = nullptr);
    ~SAManager();

    static std::shared_ptr<Logger> log() { return _log; }

private:
    void internal_init();
    void fill_menus();
    void instanciate_controllers();
    void autostart_demo();

    static std::shared_ptr<Logger> _log;

    std::unique_ptr<VoluntaryControl> _vc;
    std::unique_ptr<GeneralFormulation> _galf;
    std::unique_ptr<CompensationOptitrack> _opti;
    std::unique_ptr<RemoteComputerControl> _rm;
    std::unique_ptr<MatlabReceiver> _mr;
    std::unique_ptr<Demo> _demo;
    std::unique_ptr<SystemMonitor> _sm;

    std::unique_ptr<MenuBackend> _main_menu;
    std::unique_ptr<MenuMQTT> _menu_mqtt_binding;
    std::unique_ptr<MenuConsole> _menu_console_binding;

    std::shared_ptr<SAM::Components> _robot;

    Settings _settings;

private slots:
    void mqtt_connected_callback();
};

#endif // SAMANAGER_H
