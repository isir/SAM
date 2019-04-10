#include <QCoreApplication>

#include <iostream>

#include "control/compensationoptitrack.h"
#include "control/demo.h"
#include "control/voluntarycontrol.h"
#include "peripherals/buzzer.h"
#include "peripherals/ledstrip.h"
#include "peripherals/mcp4728.h"
#include "ui/consolemenu.h"
#include "utils/settings.h"

static QFile info_file("/var/log/sam_info");
static QFile err_file("/var/log/sam_err");

void message_handler(QtMsgType type, const QMessageLogContext& context, const QString& msg)
{
    QByteArray localMsg = msg.toLocal8Bit();
    const char* file = context.file ? context.file : "";

    switch (type) {
    case QtDebugMsg:
        info_file.write(QByteArray("Debug: ") + localMsg + " (" + QByteArray(file) + ":" + QByteArray::number(context.line) + ")\r\n");
        info_file.flush();
        break;
    case QtInfoMsg:
        info_file.write(QByteArray("Info: ") + localMsg + " (" + QByteArray(file) + ":" + QByteArray::number(context.line) + ")\r\n");
        info_file.flush();
        break;
    case QtWarningMsg:
        info_file.write(QByteArray("Warning: ") + localMsg + " (" + QByteArray(file) + ":" + QByteArray::number(context.line) + ")\r\n");
        info_file.flush();
        break;
    case QtCriticalMsg:
        err_file.write(QByteArray("Critical: ") + localMsg + " (" + QByteArray(file) + ":" + QByteArray::number(context.line) + ")\r\n");
        err_file.flush();
        break;
    case QtFatalMsg:
        err_file.write(QByteArray("Fatal: ") + localMsg + " (" + QByteArray(file) + ":" + QByteArray::number(context.line) + ")\r\n");
        err_file.flush();
        break;
    default:
        info_file.write(localMsg + " (" + QByteArray(file) + ":" + QByteArray::number(context.line) + ")\r\n");
        break;
    }
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

    Demo dm;

    buzzer_submenu.addItem(ConsoleMenuItem("Single Buzz", "sb", [&buzzer](QString) { buzzer.makeNoise(BuzzerConfig::STANDARD_BUZZ); }));
    buzzer_submenu.addItem(ConsoleMenuItem("Double Buzz", "db", [&buzzer](QString) { buzzer.makeNoise(BuzzerConfig::DOUBLE_BUZZ); }));
    buzzer_submenu.addItem(ConsoleMenuItem("Triple Buzz", "tb", [&buzzer](QString) { buzzer.makeNoise(BuzzerConfig::TRIPLE_BUZZ); }));
    menu.addItem(buzzer_submenu);

    menu.addItem(opti.menu());
    menu.addItem(vc.menu());
    menu.addItem(dm.menu());

    QObject::connect(&menu, &ConsoleMenu::finished, &a, &QCoreApplication::quit);
    menu.activate();
    dm.menu().activate();
    dm.start();

    int ret = a.exec();

    ls.set(LedStrip::none, 10);

    return ret;
}
