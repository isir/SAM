#include "utils/samanager.h"
#include <QCoreApplication>
#include <sys/mman.h>

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

    return a.exec();
}
