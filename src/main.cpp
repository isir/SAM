#include "utils/samanager.h"

int main(int argc, char* argv[])
{
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
