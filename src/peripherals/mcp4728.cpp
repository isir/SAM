#ifndef DEBUG
#define QT_NO_DEBUG_OUTPUT
#endif
#include <sys/errno.h>
#include <cstring>
#include <QCoreApplication>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>

extern "C" {
#include <i2c/smbus.h>
}

#include "mcp4728.h"

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

#define defaultVDD      5000
#define RESET           0B00000110
#define WAKE            0B00001001
#define UPDATE          0B00001000
#define MULTIWRITE      0B01000000
#define SINGLEWRITE     0B01011000
#define SEQWRITE        0B01010000
#define VREFWRITE       0B10000000
#define GAINWRITE       0B11000000
#define POWERDOWNWRITE  0B10100000
#define GENERALCALL     0B00000000
#define GAINWRITE       0B11000000

#define MAX_VALUE       4095


inline uint8_t lowByte(uint16_t word)
{
    return (uint8_t)(word & 0xFF);
}
inline uint8_t highByte(uint16_t word)
{
    return (uint8_t)(word>>8);;
}
inline uint16_t word(uint8_t highB, uint8_t lowB)
{
    return (uint16_t)((highB << 8) | lowB);
}


/**
Constructor.
Creates class object. Initialize buffers
*/
MCP4728::MCP4728(const char *deviceName, uint8_t i2cAddress)
{
    _i2cPort = open(deviceName, O_RDWR);
    if (_i2cPort == -1)
    {
        qCritical("MCP4728 ERROR : Failed to open I2C device %s. Error %d : %s.", deviceName, errno, strerror(errno));
    }
    else {
        qDebug("### MCP4728 : Open device %s", deviceName);
    }
    if (ioctl(_i2cPort, I2C_SLAVE, i2cAddress) == -1)
    {
        qCritical("MCP4728 ERROR : Failed to set address %d. Error %d : %s.", i2cAddress, errno, strerror(errno));
    }

    _vdd = defaultVDD;
    getStatus();
}

MCP4728::~MCP4728() {
    if(_i2cPort != -1) {
        close(_i2cPort);
    }
}

/*
Specific Call Reset of MCP4728 - EEPROM value will loaded to input register. refer to DATASHEET 5.4.1
*/
void MCP4728::reset() {
    i2c_smbus_write_byte(_i2cPort, RESET);
}
/*
Specific Call Wake-Up of MCP4728 - Reset Power-Down bits (PD0,PD1 = 0,0). refer to DATASHEET 5.4.2
*/
void MCP4728::wake() {
    i2c_smbus_write_byte(_i2cPort, WAKE);
}
/*
Specific Call Software update of MCP4728 - All DAC ouput update. refer to DATASHEET 5.4.3
*/
void MCP4728::update() {
    i2c_smbus_write_byte(_i2cPort, UPDATE);
}
/*
Write input register values to each channel using fastwrite method.
Values : 0-4095
*/
void MCP4728::analogWrite(uint16_t value1, uint16_t value2, uint16_t value3, uint16_t value4) {
    if(value1 > 4095 || value2 > 4095 || value3 > 4095 || value4 > 4095)
        qWarning("MCP4728 ERROR : Analog values must be in range 0 - 4095 !\n");
    else {
        _values[0] = value1;
        _values[1] = value2;
        _values[2] = value3;
        _values[3] = value4;
        qDebug("### MCP4728 Analog write : %d %d %d %d", _values[0], _values[1], _values[2], _values[3]);
        fastWrite();
    }
    update();
}
/*
Write input resister value to specified channel using fastwrite method.
Channel : 0-3, Values : 0-4095
*/
void MCP4728::analogWrite(uint8_t channel, uint16_t value) {
    if(value > 4095) {
        qWarning("MCP4728 ERROR : Analog values must be in range 0 - 4095 !\n");
        value = 4095;
    }
    _values[channel] = value;
    fastWrite();
}
/*
Write a value to specified channel using singlewrite method.
This will update both input register and EEPROM value
Channel : 0-3, Values : 0-4095
*/
void MCP4728::eepromWrite(uint8_t channel, uint16_t value)
{
    _values[channel] = value;
    _valuesEp[channel] = value;
    singleWrite(channel);
}
/*
Write values to each channel using SequencialWrite method.
This will update both input register and EEPROM value
Channel : 0-3, Values : 0-4095
*/
void MCP4728::eepromWrite(uint16_t value1, uint16_t value2, uint16_t value3, uint16_t value4)
{
    _valuesEp[0] = _values[0] = value1;
    _valuesEp[1] = _values[1] = value2;
    _valuesEp[2] = _values[2] = value3;
    _valuesEp[3] = _values[3] = value4;
    seqWrite();
}
/*
Write all input resistor values to EEPROM using SequencialWrite method.
This will update both input register and EEPROM value
This will also write current Vref, PowerDown, Gain settings to EEPROM
*/
void MCP4728::eepromWrite()
{
    seqWrite();
}
/*
Reset EEPROM and input register to factory default. Refer datasheet TABLE 4-2
Input value = 0, Voltage Reference = 1 (internal), Gain = 0, PowerDown = 0
*/
void MCP4728::eepromReset()
{
    _values[0] = 0;
    _values[1] = 0;
    _values[2] = 0;
    _values[3] = 0;
    _intVref[0] = 1;
    _intVref[1] = 1;
    _intVref[2] = 1;
    _intVref[3] = 1;
    _gain[0] = 0;
    _gain[1] = 0;
    _gain[2] = 0;
    _gain[3] = 0;
    _powerDown[0] = 0;
    _powerDown[1] = 0;
    _powerDown[2] = 0;
    _powerDown[3] = 0;
    seqWrite();
}
/*
  Write Voltage reference settings to input regiters
    Vref setting = 1 (internal), Gain = 0 (x1)   ==> Vref = 2.048V
    Vref setting = 1 (internal), Gain = 1 (x2)   ==> Vref = 4.096V
    Vref setting = 0 (external), Gain = ignored  ==> Vref = VDD
*/
void MCP4728::setVref(uint8_t value1, uint8_t value2, uint8_t value3, uint8_t value4) {
    _intVref[0] = value1;
    _intVref[1] = value2;
    _intVref[2] = value3;
    _intVref[3] = value4;
    writeVref();
}
/*
  Write Voltage reference setting to a input regiter
*/
void MCP4728::setVref(uint8_t channel, uint8_t value) {
    _intVref[channel] = value;
    writeVref();
}
/*
  Write Gain settings to input regiters
    Vref setting = 1 (internal), Gain = 0 (x1)   ==> Vref = 2.048V
    Vref setting = 1 (internal), Gain = 1 (x2)   ==> Vref = 4.096V
    Vref setting = 0 (external), Gain = ignored  ==> Vref = VDD
*/
void MCP4728::setGain(uint8_t value1, uint8_t value2, uint8_t value3, uint8_t value4) {
    _gain[0] = value1;
    _gain[1] = value2;
    _gain[2] = value3;
    _gain[3] = value4;
    writeGain();
}
/*
  Write Gain setting to a input regiter
*/
void MCP4728::setGain(uint8_t channel, uint8_t value) {
    _gain[channel] = value;
    writeGain();
}
/*
  Write Power-Down settings to input regiters
    0 = Normal , 1-3 = shut down most channel circuit, no voltage out and saving some power.
    1 = 1K ohms to GND, 2 = 100K ohms to GND, 3 = 500K ohms to GND
*/

void MCP4728::setPowerDown(uint8_t value1, uint8_t value2, uint8_t value3, uint8_t value4) {
    _powerDown[0] = value1;
    _powerDown[1] = value2;
    _powerDown[2] = value3;
    _powerDown[3] = value4;
    writePowerDown();
}
/*
  Write Power-Down setting to a input regiter
*/
void MCP4728::setPowerDown(uint8_t channel, uint8_t value) {
    _powerDown[channel] = value;
    writePowerDown();
}
/*
  Return Device ID (up to 8 devices can be used in a I2C bus, Device ID = 0-7)
*/
uint8_t MCP4728::getId()
{
    return _deviceID;
}
/*
  Return Voltage Rerference setting
*/
uint8_t MCP4728::getVref(uint8_t channel)
{
    return _intVref[channel];
}
/*
  Return Gain setting
*/
uint8_t MCP4728::getGain(uint8_t channel)
{
    return _gain[channel];
}
/*
  Return PowerDown setting
*/
uint8_t MCP4728::getPowerDown(uint8_t channel)
{
    return _powerDown[channel];
}
/*
  Return Input Regiter value
*/
uint16_t MCP4728::getValue(uint8_t channel)
{
    return _values[channel];
}
/*
  Return EEPROM Voltage Rerference setting
*/
uint8_t MCP4728::getVrefEp(uint8_t channel)
{
    return _intVrefEp[channel];
}
/*
  Return EEPROM Gain setting
*/
uint8_t MCP4728::getGainEp(uint8_t channel)
{
    return _gainEp[channel];
}
/*
  Return EEPROM PowerDown setting
*/
uint8_t MCP4728::getPowerDownEp(uint8_t channel)
{
    return _powerDownEp[channel];
}
/*
  Return EEPROM value
*/
uint16_t MCP4728::getValueEp(uint8_t channel)
{
    return _valuesEp[channel];
}
/*
  Set VDD for Vout calculation
*/
void MCP4728::vdd(uint16_t currentVdd)
{
    _vdd = currentVdd;
}
/*
  Return Vout
*/
uint16_t MCP4728::getVout(uint8_t channel)
{
    uint32_t vref;
    if (_intVref[channel] == 1) {
        vref = 2048;
    }
    else {
        vref = _vdd;
    }

    uint32_t vOut = (vref * _values[channel] * (_gain[channel] + 1)) / 4096;
    if (vOut > _vdd) {
        vOut = _vdd;
    }
    return vOut;
}
/*
  write to input register of DAC. Value(mV) (V < VDD)
*/
void MCP4728::voutWrite(uint8_t channel, uint16_t vout)
{
    _vOut[channel] = vout;
    writeVout();
}
/*
  write to input registers of DACs. Value(mV) (V < VDD)
*/
void MCP4728::voutWrite(uint16_t value1, uint16_t value2, uint16_t value3, uint16_t value4)
{
    _vOut[0] = value1;
    _vOut[1] = value2;
    _vOut[2] = value3;
    _vOut[3] = value4;
    writeVout();
}

/* _____PRIVATE FUNCTIONS_____________________________________________________ */

/*
Get current values (input register and eeprom) of MCP4728
*/
void MCP4728::getStatus()
{
    errno = 0;
    qDebug("### MCP4728 : reading all registers...");
    tcflush(_i2cPort, TCIFLUSH);

    int readBytes = read(_i2cPort, _buf, 24);
    if(readBytes == 24) {
        qDebug("### MCP4728 : status : [%s]", _buf);
        for(uint8_t seq=0; seq<8 ; seq++) {
            uint8_t deviceID = _buf[seq*3];
            uint8_t hiByte = _buf[seq*3 + 1];
            uint8_t loByte = _buf[seq*3 + 2];
            qDebug("Seq %d : [" BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN "]", seq, BYTE_TO_BINARY(deviceID), BYTE_TO_BINARY(hiByte), BYTE_TO_BINARY(loByte));

            uint8_t isEEPROM = (deviceID & 0B00001000) >> 3;
            uint8_t channel = (deviceID & 0B00110000) >> 4;
            if (isEEPROM == 1) {
                _intVrefEp[channel] = (hiByte & 0B10000000) >> 7;
                _gainEp[channel] = (hiByte & 0B00010000) >> 4;
                _powerDownEp[channel] = (hiByte & 0B01100000) >> 5;
                _valuesEp[channel] = word((hiByte & 0B00001111), loByte);
            }
            else {
                _intVref[channel] = (hiByte & 0B10000000) >> 7;
                _gain[channel] = (hiByte & 0B00010000) >> 4;
                _powerDown[channel] = (hiByte & 0B01100000) >> 5;
                _values[channel] = word((hiByte & 0B00001111), loByte);
            }
        }
    } else {
        qCritical("MCP4728 ERROR reading registers : %d byte(s) read | error %d - %s\n", readBytes, errno, strerror(errno));
    }
}
/*
FastWrite input register values - All DAC ouput update. refer to DATASHEET 5.6.1 figure 5-7
DAC Input and PowerDown bits update.
No EEPROM update
*/
void MCP4728::fastWrite() {
    for (uint8_t channel=0; channel <= 3; channel++) {
        _buf[2*channel] = highByte(_values[channel]) | ((_powerDown[channel] << 4));
        _buf[2*channel+1] = lowByte(_values[channel]);
    }
    uint8_t cmd = 0B0000000 | _buf[0]; // fastwrite cm
    i2c_smbus_write_i2c_block_data(_i2cPort, cmd, 7, &_buf[1]);
}
/*
MultiWrite input register values - All DAC ouput update. refer to DATASHEET 5.6.2
DAC Input, Gain, Vref and PowerDown bits update
No EEPROM update
*/
void MCP4728::multiWrite() {
    // TO DO IF NEEDED...
}
/*
SingleWrite input register and EEPROM - a DAC ouput update. refer to DATASHEET 5.6.4
DAC Input, Gain, Vref and PowerDown bits update
EEPROM update
*/
void MCP4728::singleWrite(uint8_t channel) {

    uint8_t cmd = SINGLEWRITE | (channel << 1);
    _buf[0] = _intVref[channel] << 7 | _powerDown[channel] << 5 | _gain[channel] << 4 | highByte(_values[channel]);
    _buf[1] = lowByte(_values[channel]);
    i2c_smbus_write_i2c_block_data(_i2cPort, cmd, 2, &_buf[0]);
}
/*
SequencialWrite input registers and EEPROM - ALL DAC ouput update. refer to DATASHEET 5.6.3
DAC Input, Gain, Vref and PowerDown bits update
EEPROM update
*/
void MCP4728::seqWrite() {
    // We only implemented seqwrite from starting channel 0.
    for (uint8_t channel=0; channel <= 3; channel++) {
        _buf[2*channel] = _intVref[channel] << 7 | _powerDown[channel] << 5 | _gain[channel] << 4 | highByte(_values[channel]);
        _buf[2*channel+1] = lowByte(_values[channel]);
    }
    uint8_t cmd = SEQWRITE;;
    i2c_smbus_write_i2c_block_data(_i2cPort, cmd, 8, &_buf[0]);
}
/*
Write Voltage reference setting to input registers. refer to DATASHEET 5.6.5
No EEPROM update
*/
void MCP4728::writeVref() {
    uint8_t cmd = VREFWRITE | (_intVref[0] << 3) | (_intVref[1] << 2) | (_intVref[2] << 1) | _intVref[3];
    i2c_smbus_write_byte(_i2cPort, cmd);
}
/*
Write Gain setting to input registers. refer to DATASHEET 5.6.7
No EEPROM update
*/
void MCP4728::writeGain() {
    uint8_t cmd = GAINWRITE | (_gain[0] << 3) | (_gain[1] << 2) | (_gain[2] << 1) | _gain[3];
    i2c_smbus_write_byte(_i2cPort, cmd);
}
/*
Write PowerDown setting to input registers. refer to DATASHEET 5.6.6
No EEPROM update
*/
void MCP4728::writePowerDown() {
    _buf[0] = POWERDOWNWRITE | (_powerDown[0] << 2) | _powerDown[1];
    _buf[1] = (_powerDown[2] << 6) | (_powerDown[3] << 4);
    i2c_smbus_write_byte_data(_i2cPort, _buf[0], _buf[1]);
}
/*
Calculate Voltage out based on current setting of Vref and gain
No EEPROM update
*/
void MCP4728::writeVout()
{
    for (uint8_t channel=0; channel <= 3; channel++) {
        if (_intVref[channel] == 1) {
            _values[channel] = _vOut[channel] / (_gain[channel] + 1) * 2;
        }
        else {
            _values[channel] = (long(_vOut[channel]) * 4096) / _vdd ;
        }
    }
    fastWrite();
}
