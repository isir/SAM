#ifndef SAMANAGER_H
#define SAMANAGER_H

#include "control/compensation_imu.h"
#include "control/compensation_optitrack.h"
#include "control/cybathlon.h"
#include "control/controle_bretelles.h"
#include "control/demo.h"
#include "control/demo_imu.h"
#include "control/jf_opti.h"
#include "control/jfimu_v1.h"
#include "control/jfimu_v3.h"
#include "control/jfimu_v4.h"
#include "control/jfoptiorientation.h"
#include "control/matlab_receiver.h"
#include "control/myo_2electrodes.h"
#include "control/pushbuttons.h"
#include "control/read_adc.h"
#include "control/recorddata.h"
#include "control/remote_computer_control.h"
#include "control/test.h"
#include "control/hololens.h"
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
    void autostart_adc();

    std::condition_variable _cv;
    std::mutex _cv_mutex;

    std::unique_ptr<VoluntaryControl> _vc;
    std::unique_ptr<pushButtons> _pb;
    std::unique_ptr<myo_2electrodes> _myo2;
    std::unique_ptr<JacobianFormulationOpti> _jfOpti;
    std::unique_ptr<JFOptiOrientation> _jfOptiOrientation;
    std::unique_ptr<JFIMU_v1> _jfIMU1;
    std::unique_ptr<JFIMU_v3> _jfIMU3;
    std::unique_ptr<JFIMU_v4> _jfIMU4;
    std::unique_ptr<RecordData> _recordData;
    std::unique_ptr<CompensationOptitrack> _opti;
    std::unique_ptr<RemoteComputerControl> _rm;
    std::unique_ptr<MatlabReceiver> _mr;
    std::unique_ptr<Demo> _demo;
    std::unique_ptr<DemoIMU> _demoimu;
    std::unique_ptr<SystemMonitor> _sm;
    std::unique_ptr<ReadADC> _adc;
    std::unique_ptr<CompensationIMU> _imu;
    std::unique_ptr<Cybathlon> _cyb;
    std::unique_ptr<Test> _test;
    std::unique_ptr<ControleBretelles> _bretelles;
    std::unique_ptr<Hololens> _hololens;

    std::unique_ptr<MenuBackend> _main_menu;
    std::unique_ptr<MenuMQTT> _menu_mqtt_binding;
    std::unique_ptr<MenuConsole> _menu_console_binding;

    std::shared_ptr<SAM::Components> _robot;
};

#endif // SAMANAGER_H
