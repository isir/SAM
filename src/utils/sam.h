#ifndef SAM_H
#define SAM_H

#include "peripherals/XIMU.h"
#include "peripherals/adafruit_ads1115.h"
#include "peripherals/buzzer.h"
#include "peripherals/helpers/osmerelbow.h"
#include "peripherals/helpers/pronosupination.h"
#include "peripherals/helpers/wristflexor.h"
#include "peripherals/ledstrip.h"
#include "peripherals/myoband/myoband.h"
#include "peripherals/touch_bionics/touch_bionics_hand.h"
#include "utils/optilistener.h"
#include <memory>

namespace SAM {
struct Components {
    std::shared_ptr<Buzzer> buzzer;
    std::shared_ptr<LedStrip> leds;
    std::shared_ptr<PronoSupination> wrist;
    std::shared_ptr<WristFlexor> flexor;
    std::shared_ptr<OsmerElbow> elbow;
    std::shared_ptr<TouchBionicsHand> hand;
    std::shared_ptr<Myoband> myoband;
    std::shared_ptr<XIMU> arm_imu;
    std::shared_ptr<XIMU> trunk_imu;
    std::shared_ptr<Adafruit_ADS1115> adc;

    std::shared_ptr<OptiListener> optitrack;
};
}

#endif // SAM_H
