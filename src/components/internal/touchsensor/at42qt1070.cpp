#include "at42qt1070.h"
#include "utils/log/log.h"
#include <fcntl.h>
#include <cstring>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <iostream>

extern "C" {
#include <i2c/smbus.h>
}

AT42QT1070::AT42QT1070(const char* deviceName)
    : MenuUser("touch", "Touch sensor", [this]() {on_exit(); })
{
    _i2cPort = open(deviceName, O_RDWR);
    if (_i2cPort == -1) {
        critical() << "AT42QT1070 ERROR : Failed to open I2C device " << deviceName << ". Error " << errno << ": " << strerror(errno);
    } else {
        info() << "### AT42QT1070 : Open device " << deviceName;
    }
    if (ioctl(_i2cPort, I2C_SLAVE, I2C_ADDRESS) == -1) {
        critical() << "AT42QT1070 ERROR : Failed to set address " << I2C_ADDRESS << ". Error " << errno << ": " << strerror(errno);
    }
    info() << "### AT42QT1070 : AT42QT1070 " << deviceName << " created at address " << static_cast<int>(I2C_ADDRESS);

    _menu->set_description("Touch sensor");
    _menu->set_code("touch");

    _menu->add_item("read", "Read Key Status", [this](std::string) {std::cout << readKeyStatus()<< std::endl; });
    _menu->add_item("calib", "Calibrate", [this](std::string) {calibrate(); });
    _menu->add_item("reset", "Reset", [this](std::string) {reset(); });
    _menu->add_item("status", "Read detection status", [this](std::string) {std::cout << readDetectionStatus() << std::endl;});

    init();
}

int32_t AT42QT1070::init() {
    int32_t err = reset();
    if (err == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        err = calibrate();
        if (err == 0) {
            err = setAKS(0,2);
            err = setAKS(3,2);
        }
    }
    return err;
}

int32_t AT42QT1070::reset()
{
    int32_t error = i2c_smbus_write_byte_data(_i2cPort, 57, 1);
    if (error == 0) {
        info() << "### AT42QT1070 : reset ok";
    } else {
        info() << "### AT42QT1070 : reset error n° " << error;
    }
    return error;
}

int32_t AT42QT1070::calibrate()
{
    int32_t error = i2c_smbus_write_byte_data(_i2cPort, 56, 1);
    if (error == 0) {
        info() << "### AT42QT1070 : calibration ok";
    } else {
        info() << "### AT42QT1070 : calibration error n° " << error;
    }
    return error;
}

int32_t AT42QT1070::readKeyStatus()
{
    int32_t value = i2c_smbus_read_byte_data(_i2cPort, 3);
    if (value < 0) {
        info() << "### AT42QT1070 : reading key status error n° " << value;
    }
    return value;
}

int32_t AT42QT1070::readKeySignal(uint8_t keyNumber)
{
    uint8_t address = keyNumber*2+4;
    int32_t value = i2c_smbus_read_word_data(_i2cPort, address);
    if (value < 0) {
        info() << "### AT42QT1070 : reading key signal error n° " << value;
    }
    return value;
}

int32_t AT42QT1070::readDetectionStatus()
{
    int32_t value = i2c_smbus_read_byte_data(_i2cPort, 2);
    if (value < 0) {
        info() << "### AT42QT1070 : reading detection status error n° " << value;
    }
    return value;
}

int32_t AT42QT1070::disableKey(uint8_t keyNumber)
{
    return setAVE(keyNumber, 0);
}

int32_t AT42QT1070::setAKS(uint8_t keyNumber, uint8_t AKS)
{
    uint8_t address = keyNumber+39;
    int32_t value = i2c_smbus_read_byte_data(_i2cPort, address);
    if (value >=0) {
        uint8_t a = (value & 0b11111100) | (AKS & 0b00000011);
        int32_t err = i2c_smbus_write_byte_data(_i2cPort, address, a);
        if (err == 0) {
            info() << "### AT42QT1070 : set AKS ok";
        } else {
            info() << "### AT42QT1070 : set AKS error n° " << err;
        }
        return err;
    } else {
        info() << "### AT42QT1070 : set AKS error n° " << value;
        return value;
    }
}

int32_t AT42QT1070::setAVE(uint8_t keyNumber, uint8_t AVE)
{
    uint8_t address = keyNumber+39;
    int32_t value = i2c_smbus_read_byte_data(_i2cPort, address);
    if (AVE>4)
        info() << "### AT42QT1070 : Specified AVE too high. Setting AVE to 3";
    if (value >=0) {
        uint8_t a = (AVE << 2) | (value & 0b00000011);
        int32_t err = i2c_smbus_write_byte_data(_i2cPort, address, a);
        if (err == 0) {
            info() << "### AT42QT1070 : set AVE ok";
        } else {
            info() << "### AT42QT1070 : set AVE error n° " << err;
        }
        return err;
    } else {
        info() << "### AT42QT1070 : set AVE error n° " << value;
        return value;
    }
}

int32_t AT42QT1070::setNTHR(uint8_t keyNumber, uint8_t NTHR)
{
    uint8_t address = keyNumber+32;
    int32_t error = i2c_smbus_write_byte_data(_i2cPort, address, NTHR);
    if (error == 0) {
        info() << "### AT42QT1070 : set NTHR ok";
    } else {
        info() << "### AT42QT1070 : set NTHR error n° " << error;
    }
    return error;
}

int32_t AT42QT1070::readAKS(uint8_t keyNumber) {
    uint8_t address = keyNumber+39;
    int32_t value = i2c_smbus_read_byte_data(_i2cPort, address);
    if (value >=0) {
        value = value & 0b00000011;
        info() << "### AT42QT1070 : AKS = " << value;
    } else {
        info() << "### AT42QT1070 : AKS error n° " << value;
    }
    return value;
}

int32_t AT42QT1070::readAVE(uint8_t keyNumber) {
    uint8_t address = keyNumber+39;
    int32_t value = i2c_smbus_read_byte_data(_i2cPort, address);
    if (value >=0) {
        value = value >> 2;
        info() << "### AT42QT1070 : AVE = " << value;
    } else {
        info() << "### AT42QT1070 : AVE error n° " << value;
    }
    return value;
}

int32_t AT42QT1070::readNTHR(uint8_t keyNumber) {
    uint8_t address = keyNumber+32;
    int32_t value = i2c_smbus_read_byte_data(_i2cPort, address);
    if (value >=0) {
        info() << "### AT42QT1070 : NTHR = " << value;
    } else {
        info() << "### AT42QT1070 : NTHR error n° " << value;
    }
    return value;
}


AT42QT1070::~AT42QT1070()
{
}
