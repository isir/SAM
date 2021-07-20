#include "epos.h"
#include <iostream>
#include <libeposcmd/Definition.h>

EPOS::EPOS(std::string name, std::string portName, unsigned short nodeId)
    : MenuUser("", "", [this]() { on_exit(); })
    , _name(name)
    , _usNodeId(nodeId)
    , _deviceName("EPOS4")
    , _protocolStackName("MAXON SERIAL V2") //MAXON_RS232, CANopen ou MAXON SERIAL V2
    , _interfaceName("RS232") //RS232, USB, CAN_ixx_usb, CAN_kvaser_usb 0, etc.
    , _portName(portName) //COM1, USB0, CAN0, etc.
    , _baudrate(115200)
{
    _menu->set_description("EPOS4 "+ name);
    _menu->set_code(name);

    if(OpenDevice() == MMC_SUCCESS) {
        //unsigned int errorCode;
        //if(PrepareDemo(&errorCode) == MMC_SUCCESS ) {
            _menu->add_item("s", "Stop", [this](std::string) { stop(); });
            _menu->add_item("g", "Go to", [this](std::string args) {if (args.length() > 0) move_to(std::stol(args)); });
            _menu->add_item("v", "Set velocity (deg/s)", [this](std::string args) {if (args.length() > 0) set_velocity(std::stol(args)); });
        //}
    } else {
        std::cout<<"Unable to open EPOS device"<< std::endl;
    }
    _menu->add_item("l", "List available interfaces", [this](std::string) { PrintAvailableInterfaces(); });
    _menu->add_item("b", "Get available baudrates", [this](std::string) {GetBaudRate();});
}

EPOS::~EPOS(){}

void EPOS::LogError(std::string functionName, int p_lResult, unsigned int p_ulErrorCode)
{
    std::cerr << _name << ": " << functionName << " failed (result=" << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< std::endl;
}

int EPOS::OpenDevice()
{
    int lResult = MMC_FAILED;
    unsigned int errorCode;

    char* pDeviceName = new char[255];
    char* pProtocolStackName = new char[255];
    char* pInterfaceName = new char[255];
    char* pPortName = new char[255];

    strcpy(pDeviceName, _deviceName.c_str());
    strcpy(pProtocolStackName, _protocolStackName.c_str());
    strcpy(pInterfaceName, _interfaceName.c_str());
    strcpy(pPortName, _portName.c_str());

    _keyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, &errorCode);

    if(_keyHandle!=nullptr && errorCode == 0)
    {
        unsigned int lBaudrate = 0;
        unsigned int lTimeout = 0;

        if(VCS_GetProtocolStackSettings(_keyHandle, &lBaudrate, &lTimeout, &errorCode)!=0)
        {
            if(VCS_SetProtocolStackSettings(_keyHandle, _baudrate, lTimeout, &errorCode)!=0)
            {
                if(VCS_GetProtocolStackSettings(_keyHandle, &lBaudrate, &lTimeout, &errorCode)!=0)
                {
                    std::cout << lBaudrate << std::endl;
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
        _keyHandle = nullptr;
    }

    delete []pDeviceName;
    delete []pProtocolStackName;
    delete []pInterfaceName;
    delete []pPortName;

    return lResult;
}

int EPOS::CloseDevice()
{
    int lResult = MMC_SUCCESS;

    unsigned int errorCode = 0;

    if(VCS_SetDisableState(_keyHandle, _usNodeId, &errorCode) == 0)
    {
        LogError("VCS_SetDisableState", lResult, errorCode);
        lResult = MMC_FAILED;
    } else {
        if(VCS_CloseDevice(_keyHandle, &errorCode)==0 || errorCode != 0)
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

    if((lResult = PrepareDemo(&p_rlErrorCode))!=MMC_SUCCESS)
	{
		LogError("PrepareDemo", lResult, p_rlErrorCode);
		return lResult;
	}

    std::cout << "set profile position mode, node = " << _usNodeId << std::endl;

    if(VCS_ActivateProfilePositionMode(_keyHandle, _usNodeId, &p_rlErrorCode) == 0)
    {
        LogError("VCS_ActivateProfilePositionMode", lResult, p_rlErrorCode);
        lResult = MMC_FAILED;
    }
    else
    {
        if (targetPosition != -1) {
            std::cout << "Moving to position = " << targetPosition << std::endl;

            if(VCS_MoveToPosition(_keyHandle, _usNodeId, targetPosition, absolute, immediately, &p_rlErrorCode) == 0)
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
    unsigned int p_rlErrorCode = 0;

    if((lResult = PrepareDemo(&p_rlErrorCode))!=MMC_SUCCESS)
	{
		LogError("PrepareDemo", lResult, p_rlErrorCode);
		return lResult;
	}

    std::cout << "set profile velocity mode, node = " << _usNodeId << std::endl;

    if(VCS_ActivateProfileVelocityMode(_keyHandle, _usNodeId, &p_rlErrorCode) == 0)
    {
        LogError("VCS_ActivateProfileVelocityMode", lResult, p_rlErrorCode);
        lResult = MMC_FAILED;
    }
    else
    {
        std::cout << "Moving with target velocity = " << targetVelocity << std::endl;

        if(VCS_MoveWithVelocity(_keyHandle, _usNodeId, targetVelocity, &p_rlErrorCode) == 0)
        {
            lResult = MMC_FAILED;
            LogError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
        }
    }

    return lResult;
}

int EPOS::PrintAvailableInterfaces()
{
	int lResult = MMC_FAILED;
	int lStartOfSelection = 1;
	int lMaxStrSize = 255;
	char* pInterfaceNameSel = new char[lMaxStrSize];
	int lEndOfSelection = 0;
	unsigned int ulErrorCode = 0;

	do
	{
		if(!VCS_GetInterfaceNameSelection((char*)_deviceName.c_str(), (char*)_protocolStackName.c_str(), lStartOfSelection, pInterfaceNameSel, lMaxStrSize, &lEndOfSelection, &ulErrorCode))
		{
			lResult = MMC_FAILED;
			LogError("GetInterfaceNameSelection", lResult, ulErrorCode);
			break;
		}
		else
		{
			lResult = MMC_SUCCESS;

			printf("interface = %s\n", pInterfaceNameSel);

			PrintAvailablePorts(pInterfaceNameSel);
		}

		lStartOfSelection = 0;
	}
	while(lEndOfSelection == 0);

	for(int i=0; i<65; i++)
	{
		std::cout << "-";
	}
	std::cout << std::endl;

	delete[] pInterfaceNameSel;

	return lResult;
}

int EPOS::PrintAvailablePorts(char* p_pInterfaceNameSel)
{
	int lResult = MMC_FAILED;
	int lStartOfSelection = 1;
	int lMaxStrSize = 255;
	char* pPortNameSel = new char[lMaxStrSize];
	int lEndOfSelection = 0;
	unsigned int ulErrorCode = 0;

	do
	{
		if(!VCS_GetPortNameSelection((char*)_deviceName.c_str(), (char*)_protocolStackName.c_str(), p_pInterfaceNameSel, lStartOfSelection, pPortNameSel, lMaxStrSize, &lEndOfSelection, &ulErrorCode))
		{
			lResult = MMC_FAILED;
			LogError("GetPortNameSelection", lResult, ulErrorCode);
			break;
		}
		else
		{
			lResult = MMC_SUCCESS;
			printf("            port = %s\n", pPortNameSel);
		}

		lStartOfSelection = 0;
	}
	while(lEndOfSelection == 0);

	return lResult;
}

int EPOS::stop()
{
    int lResult = MMC_SUCCESS;
    unsigned int p_rlErrorCode;
    std::cout << "Halt velocity movement" << std::endl;

    if(VCS_HaltVelocityMovement(_keyHandle, _usNodeId, &p_rlErrorCode) == 0)
    {
        lResult = MMC_FAILED;
        LogError("VCS_HaltVelocityMovement", lResult, p_rlErrorCode);
    }
    return lResult;
}

int EPOS::PrepareDemo(unsigned int* errorCode)
{
	int lResult = MMC_SUCCESS;
	int oIsFault = 0;

	if(VCS_GetFaultState(_keyHandle, _usNodeId, &oIsFault, errorCode) == 0)
	{
		LogError("VCS_GetFaultState", lResult, *errorCode);
		lResult = MMC_FAILED;
	}

	if(lResult==0)
	{
		if(oIsFault)
		{
			std::cout << "clear fault, node = '" << _usNodeId << std::endl;;

			if(VCS_ClearFault(_keyHandle, _usNodeId, errorCode) == 0)
			{
				LogError("VCS_ClearFault", lResult, *errorCode);
				lResult = MMC_FAILED;
			}
		}

		if(lResult==0)
		{
			int oIsEnabled = 0;

			if(VCS_GetEnableState(_keyHandle, _usNodeId, &oIsEnabled, errorCode) == 0)
			{
				LogError("VCS_GetEnableState", lResult, *errorCode);
				lResult = MMC_FAILED;
			}

			if(lResult==0)
			{
				if(!oIsEnabled)
				{
					if(VCS_SetEnableState(_keyHandle, _usNodeId, errorCode) == 0)
					{
						LogError("VCS_SetEnableState", lResult, *errorCode);
						lResult = MMC_FAILED;
					}
				}
			}
		}
	}
	return lResult;
}

void EPOS::GetBaudRate()
{
    char* pDeviceName = new char[255];
    char* pProtocolStackName = new char[255];
    char* pInterfaceName = new char[255];
    char* pPortName = new char[255];

    strcpy(pDeviceName, _deviceName.c_str());
    strcpy(pProtocolStackName, _protocolStackName.c_str());
    strcpy(pInterfaceName, _interfaceName.c_str());
    strcpy(pPortName, _portName.c_str());

    unsigned int baudrateSel;
    int endOfSelection = false;
    unsigned int errorCode = 0;

    if(VCS_GetBaudrateSelection(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, true, &baudrateSel, &endOfSelection, &errorCode))
    {
        while(!endOfSelection)
        {
            VCS_GetBaudrateSelection(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, false, &baudrateSel, &endOfSelection, &errorCode);
            std::cout << baudrateSel << std::endl;
        } 
    }
}