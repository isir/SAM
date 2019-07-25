#include "utils/samanager.h"
#include <QCoreApplication>
#include <espeak/speak_lib.h>
#include <malloc.h>
#include <sys/mman.h>

#define MEMORY_POOL (100 * 1024 * 1024) // 100MB

int main(int argc, char* argv[])
{
    // Now lock all current and future pages from preventing of being paged
    mlockall(MCL_CURRENT | MCL_FUTURE);
    // Turn off malloc trimming.
    mallopt(M_TRIM_THRESHOLD, -1);
    // Turn off mmap usage.
    mallopt(M_MMAP_MAX, 0);

    void* buf = malloc(MEMORY_POOL);
    free(buf);

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
