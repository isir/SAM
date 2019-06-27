#ifndef SAMANAGER_H
#define SAMANAGER_H

#include "control/compensation_imu.h"
#include "control/compensation_optitrack.h"
#include "control/demo.h"
#include "control/matlab_receiver.h"
#include "control/remote_computer_control.h"
#include "control/voluntary_control.h"
#include "ui/console_menu.h"
#include "utils/logger.h"
#include "utils/sam.h"
#include "utils/settings.h"
#include "utils/system_monitor.h"
#include <QCoreApplication>
#include <QMqttClient>
#include <QObject>

class SAManager : public QObject {
    Q_OBJECT
public:
    explicit SAManager(QCoreApplication* a, QObject* parent = nullptr);
    ~SAManager();

    static std::shared_ptr<Logger> log() { return _log; }

private:
    static std::shared_ptr<QMqttClient> _mqtt;
    static std::shared_ptr<Logger> _log;

    std::shared_ptr<SystemMonitor> _sm;

    std::shared_ptr<VoluntaryControl> _vc;
    std::shared_ptr<CompensationOptitrack> _opti;
    std::shared_ptr<CompensationIMU> _imu;
    std::shared_ptr<RemoteComputerControl> _rm;
    std::shared_ptr<MatlabReceiver> _mr;
    std::shared_ptr<Demo> _demo;

    std::shared_ptr<ConsoleMenu> _main_menu;
    std::shared_ptr<ConsoleMenu> _buzzer_submenu;

    SAM::Components _robot;

    Settings _settings;

private slots:
    void mqtt_connected_callback();
};

#endif // SAMANAGER_H
