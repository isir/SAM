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
    EPOS(std::string name, std::string portName, unsigned short nodeId);
    virtual ~EPOS();
    void LogError(std::string functionName, int p_lResult, unsigned int p_ulErrorCode);
    int OpenDevice();
    int CloseDevice();
    int move_to(long targetPosition, bool absolute=false, bool immediately=true);
    int set_velocity(long targetVelocity);
    int PrintAvailableInterfaces();
    int PrintAvailablePorts(char* p_pInterfaceNameSel);
    int stop();
    int PrepareDemo(unsigned int* eErrorCode);

protected:
    virtual void on_exit(){};

private:
    std::string _name;
    void* _keyHandle = 0;
    unsigned short _usNodeId = 1;
    std::string _deviceName;
    std::string _protocolStackName;
    std::string _interfaceName;
    std::string _portName;
    unsigned int _baudrate = 0;
};

#endif // EPOS_H
