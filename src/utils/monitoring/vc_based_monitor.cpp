#include "vc_based_monitor.h"
#include "utils/log/log.h"

namespace Monitoring {

VCBasedMonitor::VCBasedMonitor()
{
    vcos_init();

    if (vchi_initialise(&_vchi_instance) != 0) {
        critical() << "VCHI initialization failed";
    }
    if (vchi_connect(nullptr, 0, _vchi_instance) != 0) {
        critical() << "VCHI connection failed";
    }
    vc_vchi_gencmd_init(_vchi_instance, &_vchi_connection, 1);
}

VCBasedMonitor::~VCBasedMonitor()
{
    vc_gencmd_stop();
    vchi_disconnect(_vchi_instance);
}

std::string VCBasedMonitor::vc_send_receive(std::string cmd)
{
    int ret;
    char vc_buf[128];

    if ((ret = vc_gencmd_send(cmd.c_str())) != 0) {
        warning() << "vc_gencmd_send returned " << ret;
    }
    if ((ret = vc_gencmd_read_response(vc_buf, 128)) != 0) {
        warning() << "vc_gencmd_read_response returned " << ret;
    }

    return std::string(vc_buf);
}

}
