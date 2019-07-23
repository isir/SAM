#include "utils/samanager.h"
#include <QCoreApplication>
#include <sys/mman.h>

#include <espeak/speak_lib.h>

int main(int argc, char* argv[])
{
    mlockall(MCL_CURRENT | MCL_FUTURE);

    QCoreApplication::setOrganizationName("ISIR");
    QCoreApplication::setOrganizationDomain("isir.upmc.fr");
    QCoreApplication::setApplicationName("SAM");

    QCoreApplication a(argc, argv);

    SAManager sam(&a);

    {
        Settings dummy;
        qInfo() << "Using settings from " << dummy.fileName();
    }

    espeak_Initialize(AUDIO_OUTPUT_PLAYBACK, 0, nullptr, 0);
    espeak_Synth("Starting.", 10, 0, POS_CHARACTER, 10, espeakCHARS_AUTO, nullptr, nullptr);

    return a.exec();
}
