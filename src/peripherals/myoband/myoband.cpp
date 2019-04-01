#include "myoband.h"
#include "localfeatures.h"
#include "unistd.h"

using namespace myolinux;

/**
 * \brief Myoband::Myoband Constructor
 * \param serial the USB stick port
 */
Myoband::Myoband(Serial serial) : Client(serial) {
    _emgs = new int[_NB_EMGS];

    _old_emgs = new int*[_NB_EMGS];
    for(int i=0; i<_NB_EMGS; i++) {
        _old_emgs[i] = new int[_WIN_SIZE];
        for(int j=0; j<_WIN_SIZE; j++)
            _old_emgs[i][j] = 0;
    }

    _mean_emgs = new float[_NB_EMGS];
    _rms = new float[_NB_EMGS];
    _imus = new float[_NB_IMUS];
}

/**
 * \brief Myoband::~Myoband Destructor
 */
Myoband::~Myoband() {
    delete _emgs;
    delete _mean_emgs;
    delete _imus;
    for(int i=0; i<_NB_EMGS ; i++)
        delete _old_emgs[i];
    delete _old_emgs;
    delete _rms;
}

/**
 * \brief Myoband::startListen Launchs the listening thread.
 */
void Myoband::startListen() {
    _stop_thread = false;
    this->StartInternalThread();
}

/**
 * \brief Myoband::stopListen Stops the listening thread.
 */
void Myoband:: stopListen() {
    _stop_thread = true;
}

/**
 * \brief Myoband::InternalThreadEntry Is the listening thread.
 */
void Myoband::InternalThreadEntry() {

    using namespace std::placeholders;

    onEmg(std::bind(&Myoband::_update_emgs, this, _1));
    onImu(std::bind(&Myoband::_update_imus, this, _1, _2, _3));

    while(!_stop_thread) {
        this->listen();
    }

}

/**
 * \brief Myoband::_update_emgs EMGs callback. Executed when receiving new emgs from our Myoband's friend.
 * \param sample
 */
void Myoband::_update_emgs(EmgSample sample) {
    // Update EMGs
    for (unsigned i = 0; i < 8; i++)
        _emgs[i] = sample[i];

    // Update window
    for(int i=0; i<_NB_EMGS ; i++) {
        for(int j=_WIN_SIZE-1; j>0; j--) {
            _old_emgs[i][j] = _old_emgs[i][j-1];
        }
        _old_emgs[i][0] = _emgs[i];
    }

    // Extract RMS
    for(int i=0 ; i<_NB_EMGS ; i++) {
        _rms[i] = (float) extractRMS(_old_emgs[i], _WIN_SIZE);
    }
}

/**
 * \brief Myoband::_update_imus IMUs callback. Executed when receiving new IMUs data from Myoband.
 * \param ori
 * \param acc
 * \param gyr
 */
void Myoband::_update_imus(OrientationSample ori, AccelerometerSample acc, GyroscopeSample gyr) {
    for(unsigned i=0 ; i<4 ; i++)
        _imus[i] = (float) ori[i] / OrientationScale;
    for(unsigned i=0 ; i<3 ; i++)
        _imus[4+i] = (float) acc[i] / AccelerometerScale;
    for(unsigned i=0 ; i<3 ; i++)
        _imus[7+i] = (float) gyr[i] / GyroscopeScale;
}

/**
 * \brief Myoband::connect Trys to connect the Myoband via Bluetooth.
 * You can help it find it by pluging in/unpluging a powered micro USB cable into the Myoband.
 */
void Myoband::connect() {
    printf("MYOBAND INFO : Trying to connect... Try to plug in/unplug the USB port\n");
    return  Client::connect();
}
