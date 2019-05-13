#ifndef SAM_H
#define SAM_H

#include "peripherals/XIMU.h"
#include "peripherals/actuators/actuator.h"
#include "peripherals/actuators/wrist_flexor.h"
#include "peripherals/adafruit_ads1115.h"
#include "peripherals/buzzer.h"
#include "peripherals/ledstrip.h"
#include "peripherals/myoband/myoband.h"
#include "peripherals/touch_bionics/touch_bionics_hand.h"
#include "utils/opti_listener.h"
#include <memory>

namespace SAM {
struct Components {
    std::shared_ptr<Buzzer> buzzer;
    std::shared_ptr<LedStrip> leds;
    std::shared_ptr<Actuator> wrist_pronosup;
    std::shared_ptr<WristFlexor> wrist_flex;
    std::shared_ptr<Actuator> elbow;
    std::shared_ptr<TouchBionicsHand> hand;
    std::shared_ptr<Myoband> myoband;
    std::shared_ptr<XIMU> arm_imu;
    std::shared_ptr<XIMU> trunk_imu;
    std::shared_ptr<Adafruit_ADS1115> adc;

    std::shared_ptr<OptiListener> optitrack;
};
}

#endif // SAM_H
