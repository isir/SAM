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
#include "utils/mqttclient.h"
#include "utils/settings.h"

static QFile info_file("/var/log/sam_info");
static QFile err_file("/var/log/sam_err");

void message_handler(QtMsgType type, const QMessageLogContext& context, const QString& msg)
{
    static MqttClient& mqtt = MqttClient::instance();

    QByteArray localMsg = msg.toLocal8Bit();

    QFile* log_file = &info_file;
    QString mqtt_topic_name = "sam/log/";
    QByteArray line;
    QByteArray suffix = " (" + QByteArray(context.file ? context.file : "") + ":" + QByteArray::number(context.line) + ")\r\n";

    switch (type) {
    case QtDebugMsg:
        line = "Debug: ";
        mqtt_topic_name += "debug";
        break;
    case QtInfoMsg:
        line = "Info: ";
        mqtt_topic_name += "info";
        break;
    case QtWarningMsg:
        line = "Warning: ";
        mqtt_topic_name += "warning";
        break;
    case QtCriticalMsg:
        line = "Critical: ";
        mqtt_topic_name += "critical";
        log_file = &err_file;
        break;
    case QtFatalMsg:
        line = "Fatal: ";
        mqtt_topic_name += "fatal";
        log_file = &err_file;
        break;
    default:
        break;
    }
    line += localMsg + suffix;
    log_file->write(line);
    mqtt.publish(mqtt_topic_name, line);
}

int main(int argc, char* argv[])
{
    info_file.open(QIODevice::ReadWrite | QIODevice::Truncate);
    err_file.open(QIODevice::ReadWrite | QIODevice::Truncate);

    qInstallMessageHandler(message_handler);
    QCoreApplication a(argc, argv);

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

    //Demo dm;

    buzzer_submenu.addItem(ConsoleMenuItem("Single Buzz", "sb", [&buzzer](QString) { buzzer.makeNoise(BuzzerConfig::STANDARD_BUZZ); }));
    buzzer_submenu.addItem(ConsoleMenuItem("Double Buzz", "db", [&buzzer](QString) { buzzer.makeNoise(BuzzerConfig::DOUBLE_BUZZ); }));
    buzzer_submenu.addItem(ConsoleMenuItem("Triple Buzz", "tb", [&buzzer](QString) { buzzer.makeNoise(BuzzerConfig::TRIPLE_BUZZ); }));
    menu.addItem(buzzer_submenu);

    menu.addItem(opti.menu());
    menu.addItem(vc.menu());
    menu.addItem(rm.menu());
    //menu.addItem(dm.menu());

    QObject::connect(&menu, &ConsoleMenu::finished, &a, &QCoreApplication::quit);
    menu.activate();
    //dm.menu().activate();
    //dm.start();

    QObject::connect(&menu, &ConsoleMenu::finished, &a, &QCoreApplication::quit);
    menu.activate();

    try {
        Demo dm;
        menu.addItem(dm.menu());
        dm.menu().activate();
        dm.start();
    } catch (std::exception& e) {
        qCritical() << e.what();
    }

    int ret = a.exec();

    ls.set(LedStrip::none, 10);

    return ret;
}
