#ifndef EPOS_H
#define EPOS_H

#include "utils/interfaces/menu_user.h"
#include <string.h>

#ifndef MMC_SUCCESS
    #define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
    #define MMC_FAILED 1
#endif

class EPOS : public MenuUser {
public:
    EPOS();
    virtual ~EPOS();
    void LogError(std::string functionName, int p_lResult, unsigned int p_ulErrorCode);
    int   OpenDevice();
    int   CloseDevice(unsigned int* p_pErrorCode);
    int   move_to(long targetPosition, bool absolute=false, bool immediately=true);
    int   set_velocity(long targetVelocity);
    int   stop();

protected:
    virtual void on_exit(){};

private:
//    std::string _name;
    void* _pKeyHandle = 0;
    unsigned short _usNodeId = 1;
    std::string _deviceName;
    std::string _protocolStackName;
    std::string _interfaceName;
    std::string _portName;
    unsigned int _baudrate = 0;
    const std::string _programName = "LinuxEposCmd";
};

#endif // EPOS_H
