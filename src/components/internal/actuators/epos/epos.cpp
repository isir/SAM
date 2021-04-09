#include "epos.h"
#include <iostream>
#include "Definitions.h"

EPOS::EPOS()
    : MenuUser("", "", [this]() { on_exit(); })
    , _usNodeId(1)
    , _deviceName("EPOS4")
    , _protocolStackName("MAXON SERIAL V2") //MAXON_RS232, CANopen ou MAXON SERIAL V2
    , _interfaceName("USB") //RS232, USB, CAN_ixx_usb, CAN_kvaser_usb 0, etc.
    , _portName("USB0") //COM1, USB0, CAN0, etc.
    , _baudrate(1000000)
{
    _menu->set_description("EPOS4");
    _menu->set_code("pronosup");

    OpenDevice();

    _menu->add_item("s", "Stop", [this](std::string) { stop(); });
    _menu->add_item("g", "Go to", [this](std::string args) {if (args.length() > 0) move_to(std::stol(args)); });
    _menu->add_item("v", "Set velocity (deg/s)", [this](std::string args) { if (args.length() > 0) args = "0"; set_velocity(std::stol(args)); });
}

EPOS::~EPOS(){}

void EPOS::LogError(std::string functionName, int p_lResult, unsigned int p_ulErrorCode)
{
    std::cerr << _programName << ": " << functionName << " failed (result=" << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< std::endl;
}

int EPOS::OpenDevice()
{
    int lResult = MMC_FAILED;
    unsigned int p_pErrorCode;

    char* pDeviceName = new char[255];
    char* pProtocolStackName = new char[255];
    char* pInterfaceName = new char[255];
    char* pPortName = new char[255];

    strcpy(pDeviceName, _deviceName.c_str());
    strcpy(pProtocolStackName, _protocolStackName.c_str());
    strcpy(pInterfaceName, _interfaceName.c_str());
    strcpy(pPortName, _portName.c_str());

    _pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, &p_pErrorCode);

    if(_pKeyHandle!=nullptr && p_pErrorCode == 0)
    {
        unsigned int lBaudrate = 0;
        unsigned int lTimeout = 0;

        if(VCS_GetProtocolStackSettings(_pKeyHandle, &lBaudrate, &lTimeout, &p_pErrorCode)!=0)
        {
            if(VCS_SetProtocolStackSettings(_pKeyHandle, _baudrate, lTimeout, &p_pErrorCode)!=0)
            {
                if(VCS_GetProtocolStackSettings(_pKeyHandle, &lBaudrate, &lTimeout, &p_pErrorCode)!=0)
                {
                    if(_baudrate==lBaudrate)
                    {
                        lResult = MMC_SUCCESS;
                    }
                }
            }
        }
    }
    else
    {
        _pKeyHandle = nullptr;
    }

    delete []pDeviceName;
    delete []pProtocolStackName;
    delete []pInterfaceName;
    delete []pPortName;

    return lResult;
}

int EPOS::CloseDevice(unsigned int* p_pErrorCode)
{
    int lResult = MMC_SUCCESS;

    *p_pErrorCode = 0;

    if(VCS_SetDisableState(_pKeyHandle, _usNodeId, p_pErrorCode) == 0)
    {
        LogError("VCS_SetDisableState", lResult, *p_pErrorCode);
        lResult = MMC_FAILED;
    } else {
        if(VCS_CloseDevice(_pKeyHandle, p_pErrorCode)==0 || *p_pErrorCode != 0)
        {
            lResult = MMC_FAILED;
        }
    }

    return lResult;
}

int EPOS::move_to(long targetPosition, bool absolute, bool immediately)
{
    int lResult = MMC_SUCCESS;
    unsigned int p_rlErrorCode=0;

    std::cout << "set profile position mode, node = " << _usNodeId << std::endl;

    if(VCS_ActivateProfilePositionMode(_pKeyHandle, _usNodeId, &p_rlErrorCode) == 0)
    {
        LogError("VCS_ActivateProfilePositionMode", lResult, p_rlErrorCode);
        lResult = MMC_FAILED;
    }
    else
    {
        if (targetPosition != -1) {
            std::cout << "Moving to position = " << targetPosition << std::endl;

            if(VCS_MoveToPosition(_pKeyHandle, _usNodeId, targetPosition, absolute, immediately, &p_rlErrorCode) == 0)
            {
                LogError("VCS_MoveToPosition", lResult, p_rlErrorCode);
                lResult = MMC_FAILED;
            }

        }
    }

    return lResult;
}

int EPOS::set_velocity(long targetVelocity)
{
    int lResult = MMC_SUCCESS;
    unsigned int p_rlErrorCode;

    std::cout << "set profile velocity mode, node = " << _usNodeId << std::endl;

    if(VCS_ActivateProfileVelocityMode(_pKeyHandle, _usNodeId, &p_rlErrorCode) == 0)
    {
        LogError("VCS_ActivateProfileVelocityMode", lResult, p_rlErrorCode);
        lResult = MMC_FAILED;
    }
    else
    {
        std::cout << "Moving with target velocity = " << targetVelocity << std::endl;

        if(VCS_MoveWithVelocity(_pKeyHandle, _usNodeId, targetVelocity, &p_rlErrorCode) == 0)
        {
            lResult = MMC_FAILED;
            LogError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
        }
    }

    return lResult;
}

int EPOS::stop()
{
    int lResult = MMC_SUCCESS;
    unsigned int p_rlErrorCode;
    std::cout << "Halt velocity movement" << std::endl;

    if(VCS_HaltVelocityMovement(_pKeyHandle, _usNodeId, &p_rlErrorCode) == 0)
    {
        lResult = MMC_FAILED;
        LogError("VCS_HaltVelocityMovement", lResult, p_rlErrorCode);
    }
    return lResult;
}
