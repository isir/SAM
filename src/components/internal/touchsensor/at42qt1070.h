#ifndef AT42QT1070_H
#define AT42QT1070_H

#include <stdint.h>
#include "utils/interfaces/menu_user.h"

// I2C address
#define I2C_ADDRESS		0x1B

class AT42QT1070 :  public MenuUser
{
public:
    AT42QT1070(const char* deviceName);
    virtual ~AT42QT1070();
    //Functions return a negative number in case of error.
    // In case of success, they return 0 for write functions, and the read value for read functions.
    int32_t init();
    int32_t reset();
    int32_t calibrate();
    int32_t readKeyStatus();
    int32_t readKeySignal(uint8_t keyNumber);
    int32_t readDetectionStatus();
    int32_t disableKey(uint8_t keyNumber);
    int32_t setAKS(uint8_t keyNumber, uint8_t AKS);
    int32_t setAVE(uint8_t keyNumber, uint8_t AVE);
    int32_t setNTHR(uint8_t keyNumber, uint8_t NTHR);
    int32_t readAKS(uint8_t keyNumber);
    int32_t readAVE(uint8_t keyNumber);
    int32_t readNTHR(uint8_t keyNumber);

protected:
    virtual void on_exit(){};

private:
    int _i2cPort;
};

#endif // AT42QT1070_H
