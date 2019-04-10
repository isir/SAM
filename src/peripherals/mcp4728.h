#ifndef MCP4728_H
#define MCP4728_H

#include <stdint.h>

class MCP4728 {

public:
    MCP4728(const char* deviceName, uint8_t i2cAddress);
    ~MCP4728();
    void vdd(uint16_t);
    void reset();
    void wake();
    void update();
    void analogWrite(uint16_t, uint16_t, uint16_t, uint16_t);
    void analogWrite(uint8_t, uint16_t);
    void eepromWrite(uint16_t, uint16_t, uint16_t, uint16_t);
    void eepromWrite(uint8_t, uint16_t);
    void eepromWrite();
    void eepromReset();
    void print();
    void setVref(uint8_t, uint8_t, uint8_t, uint8_t);
    void setVref(uint8_t, uint8_t);
    void setGain(uint8_t, uint8_t, uint8_t, uint8_t);
    void setGain(uint8_t, uint8_t);
    void setPowerDown(uint8_t, uint8_t, uint8_t, uint8_t);
    void setPowerDown(uint8_t, uint8_t);
    uint8_t getId();
    uint8_t getVref(uint8_t);
    uint8_t getGain(uint8_t);
    uint8_t getPowerDown(uint8_t);
    uint16_t getValue(uint8_t);
    uint8_t getVrefEp(uint8_t);
    uint8_t getGainEp(uint8_t);
    uint8_t getPowerDownEp(uint8_t);
    uint16_t getValueEp(uint8_t);
    uint16_t getVout(uint8_t);
    void voutWrite(uint8_t, uint16_t);
    void voutWrite(uint16_t, uint16_t, uint16_t, uint16_t);
    void getStatus();

private:
    int _i2cPort;
    void fastWrite();
    void multiWrite();
    void singleWrite(uint8_t);
    void seqWrite();
    void writeVref();
    void writeGain();
    void writePowerDown();
    void writeVout();
    uint8_t _dev_address;
    uint8_t _deviceID;
    uint8_t _intVref[4];
    uint8_t _gain[4];
    uint8_t _powerDown[4];
    uint16_t _values[4];
    uint16_t _valuesEp[4];
    uint8_t _intVrefEp[4];
    uint8_t _gainEp[4];
    uint8_t _powerDownEp[4];
    uint16_t _vOut[4];
    uint16_t _vdd;
    uint8_t _buf[30];
};

#endif // MCP4728_H
