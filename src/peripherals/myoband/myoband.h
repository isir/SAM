#ifndef MYOBAND_H
#define MYOBAND_H

#include "myoLinux/myoclient.h"
#include "myoLinux/serial.h"
#include <QMutex>
#include <control/basiccontroller.h>

class Myoband : public BasicController {

public:
    Myoband();
    ~Myoband();

    bool setup();
    void loop(double dt, double time);
    void cleanup();

    bool connected() { return _client.connected(); }

    /**
     * @brief getEMGs
     * @return Pointer of 8 EMGs i nrange [-127 ; 128].
     */
    int* getEMGs() { return _emgs; }
    /**
     * @brief getIMUs
     * @return Pointer of 10 IMUs values : [ 4 Quats ; 3 Accs ; 3 Gyros].
     */
    float* getIMUs() { return _imus; }
    /**
     * @brief getAccs
     * @return Pointer of 3 Accs in g.
     */
    float* getAccs() { return &_imus[4]; }
    /**
     * @brief getQuats
     * @return Pointer of 4 Quats in std units.
     */
    float* getQuats() { return _imus; }
    /**
     * @brief getGyros
     * @return Pointer of 3 Gyros in deg/s (to check).
     */
    float* getGyros() { return &_imus[7]; }

private:
    myolinux::myo::Client _client;
    QMutex _mutex;

    static const int _NB_EMGS = 8;
    static const int _NB_IMUS = 10;

    int _emgs[_NB_EMGS];
    float _imus[_NB_IMUS];
};

#endif // MYOBAND_H
