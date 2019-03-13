#include <QCoreApplication>
#include <QRandomGenerator>

#include <iostream>

#include "ui/consolemenu.h"
#include "peripherals/buzzer.h"
#include "peripherals/mcp4728.h"
#include "control/voluntarycontrol.h"
#include "control/compensationoptitrack.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    QCoreApplication::setOrganizationName("ISIR");
    QCoreApplication::setOrganizationDomain("isir.upmc.fr");
    QCoreApplication::setApplicationName("SAM");

    {
        QSettings dummy;
        std::cout << "Using settings from " << dummy.fileName().toStdString() << std::endl;
    }

    wiringPiSetup();

    QRandomGenerator rng;

    MCP4728 dac0("/dev/i2c-1", 0x60);
    MCP4728 dac1("/dev/i2c-1", 0x61);
    dac0.analogWrite(0,0,0,0);
    dac1.analogWrite(0,0,0,0);

    Buzzer buzzer(29);
    ConsoleMenu menu("Main menu","main");
    ConsoleMenu buzzer_submenu("Buzzer submenu","buzzer");
    ConsoleMenu dac_submenu("DAC submenu","dac");

    VoluntaryControl vc;
    CompensationOptitrack opti;

    buzzer_submenu.addItem(ConsoleMenuItem("Single Buzz","sb",[&buzzer](QString){ buzzer.makeNoise(BuzzerConfig::STANDARD_BUZZ); }));
    buzzer_submenu.addItem(ConsoleMenuItem("Double Buzz","db",[&buzzer](QString){ buzzer.makeNoise(BuzzerConfig::DOUBLE_BUZZ); }));
    buzzer_submenu.addItem(ConsoleMenuItem("Triple Buzz","tb",[&buzzer](QString){ buzzer.makeNoise(BuzzerConfig::TRIPLE_BUZZ); }));
    menu.addItem(buzzer_submenu);

    dac_submenu.addItem(ConsoleMenuItem("Random Values","r",[&dac0,&dac1,&rng](QString){ dac0.analogWrite(rng.bounded(0,4096),rng.bounded(0,4096),rng.bounded(0,4096),rng.bounded(0,4096)); dac1.analogWrite(rng.bounded(0,4096),rng.bounded(0,4096),rng.bounded(0,4096),rng.bounded(0,4096)); }));
    menu.addItem(dac_submenu);

    menu.addItem(opti.menu());
    menu.addItem(vc.menu());

    QObject::connect(&menu,&ConsoleMenu::finished,&a,&QCoreApplication::quit);
    menu.activate();

    int ret = a.exec();

    dac0.analogWrite(0,0,0,0);
    dac1.analogWrite(0,0,0,0);

    return ret;
}
