#ifndef MYOBAND_H
#define MYOBAND_H

#include <pthread.h>
#include "MyThreadClass.h"
#include "myoLinux/myoclient.h"
#include "myoLinux/serial.h"

using namespace myolinux::myo;

/**
 * \brief The Myoband class creates an instance to listen to the Myoband values.\n
 * Use it as follow :
 * - connect() until connected()
 * - setSleepMode() and setMode()
 * - Finally startListen()
 * - stopListen() and disconnect() after use
 * \n
 * \warning - vibrate() is not yet integrated. Please implemente it before using it.
 * \warning - When adding it in project, please add LocalFeatures lib BEFORE Myoband lib
 *
 */
class Myoband : public Client, public MyThreadClass
{

public:
    Myoband(myolinux::Serial serial);
    ~Myoband();
    void        connect();
    void        startListen();
    void        stopListen();
    /**
     * @brief getEMGs
     * @return Pointer of 8 EMGs i nrange [-127 ; 128].
     */
    int*        getEMGs(){return _emgs;}
    /**
     * @brief getRMSs
     * @return Pointer of 8 RMSs in range [0 ; 128].
     */
    float*      getRMSs() {return _rms;}
    /**
     * @brief getIMUs
     * @return Pointer of 10 IMUs values : [ 4 Quats ; 3 Accs ; 3 Gyros].
     */
    float*      getIMUs(){return _imus;}
    /**
     * @brief getAccs
     * @return Pointer of 3 Accs in g.
     */
    float*      getAccs(){return &_imus[4];}
    /**
     * @brief getQuats
     * @return Pointer of 4 Quats in std units.
     */
    float*      getQuats(){return _imus;}
    /**
     * @brief getGyros
     * @return Pointer of 3 Gyros in deg/s (to check).
     */
    float*      getGyros(){return &_imus[7];}

private:
    void        InternalThreadEntry();
    void        _update_imus(OrientationSample ori, AccelerometerSample acc, GyroscopeSample gyr);
    void        _update_emgs(EmgSample sample);
    bool        _stop_thread;

    const int   _NB_EMGS = 8;
    const int   _NB_IMUS = 10;
    const int   _WIN_SIZE = 20;
    int*        _emgs;
    int**       _old_emgs;
    float*      _mean_emgs;
    float*      _rms;
    float*      _imus;


};

#endif // MYOBAND_H
