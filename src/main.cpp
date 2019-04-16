#include <QCoreApplication>

#include <iostream>

#include "control/compensationoptitrack.h"
#include "control/demo.h"
#include "control/remotecomputercontrol.h"
#include "control/voluntarycontrol.h"
#include "peripherals/buzzer.h"
#include "peripherals/ledstrip.h"
#include "peripherals/mcp4728.h"
#include "ui/consolemenu.h"
#include "utils/logger.h"
#include "utils/settings.h"
#include "utils/systemmonitor.h"

void message_handler(QtMsgType type, const QMessageLogContext& context, const QString& msg)
{
    static Logger l;
    l.async_handle_message(type, context, msg);
}

int main(int argc, char* argv[])
{
    QCoreApplication a(argc, argv);

    QEventLoop el;
    MqttClient& mqtt = MqttClient::instance();
    qInfo() << "Connecting to MQTT broker...";
    QObject::connect(&mqtt, &QMqttClient::connected, &el, &QEventLoop::quit);
    el.exec();
    qInfo() << "Connected to MQTT broker.";

    qInstallMessageHandler(&message_handler);

    SystemMonitor sm;

    QCoreApplication::setOrganizationName("ISIR");
    QCoreApplication::setOrganizationDomain("isir.upmc.fr");
    QCoreApplication::setApplicationName("SAM");

    {
        Settings dummy;
        qInfo() << "Using settings from " << dummy.fileName();
    }

    wiringPiSetup();

    Buzzer buzzer(29);
    ConsoleMenu menu("Main menu", "main");
    ConsoleMenu buzzer_submenu("Buzzer submenu", "buzzer");

    LedStrip& ls = LedStrip::instance();
    ls.set(LedStrip::white, 10);

    VoluntaryControl vc;
    CompensationOptitrack opti;
    RemoteComputerControl rm;

    buzzer_submenu.addItem(ConsoleMenuItem("Single Buzz", "sb", [&buzzer](QString) { buzzer.makeNoise(BuzzerConfig::STANDARD_BUZZ); }));
    buzzer_submenu.addItem(ConsoleMenuItem("Double Buzz", "db", [&buzzer](QString) { buzzer.makeNoise(BuzzerConfig::DOUBLE_BUZZ); }));
    buzzer_submenu.addItem(ConsoleMenuItem("Triple Buzz", "tb", [&buzzer](QString) { buzzer.makeNoise(BuzzerConfig::TRIPLE_BUZZ); }));
    menu.addItem(buzzer_submenu);

    menu.addItem(opti.menu());
    menu.addItem(vc.menu());
    menu.addItem(rm.menu());

    QObject::connect(&menu, &ConsoleMenu::finished, &a, &QCoreApplication::quit);
    menu.activate();

    Demo* dm = nullptr;
    try {
        dm = new Demo();
        menu.addItem(dm->menu());
        dm->menu().activate();
        dm->start();
    } catch (std::exception& e) {
        qCritical() << e.what();
    }

    int ret = a.exec();

    ls.set(LedStrip::none, 10);
    delete dm;
    return ret;
}
