#ifndef VCBASEDMONITOR_H
#define VCBASEDMONITOR_H

#include "abstract_monitor.h"

extern "C" {
#include "interface/vmcs_host/vc_vchi_gencmd.h"
}

namespace Monitoring {

class VCBasedMonitor : public AbstractMonitor
{
public:
    VCBasedMonitor();
    virtual ~VCBasedMonitor() override;

    virtual void update() override = 0;
    virtual std::string formatted_output() override = 0;

protected:
    std::string vc_send_receive(std::string cmd);

    VCHI_INSTANCE_T _vchi_instance;
    VCHI_CONNECTION_T* _vchi_connection;
};

}

#endif // VCBASEDMONITOR_H
