#ifndef SAM_H
#define SAM_H

#include "peripherals/XIMU.h"
#include "peripherals/actuators/actuator.h"
#include "peripherals/actuators/custom_elbow.h"
#include "peripherals/actuators/osmer_elbow.h"
#include "peripherals/actuators/pronosupination.h"
#include "peripherals/actuators/shoulder_rotator.h"
#include "peripherals/actuators/wrist_flexor.h"
#include "peripherals/actuators/wrist_rotator.h"
#include "peripherals/adafruit_ads1115.h"
#include "peripherals/buzzer.h"
#include "peripherals/ledstrip.h"
#include "peripherals/myoband/myoband.h"
#include "peripherals/touch_bionics/touch_bionics_hand.h"
#include "utils/opti_listener.h"
#include <memory>

namespace SAM {
class Sensors {
public:
    Sensors();

    std::unique_ptr<Myoband> myoband;
    std::unique_ptr<XIMU> arm_imu;
    std::unique_ptr<XIMU> trunk_imu;
    std::unique_ptr<Adafruit_ADS1115> adc;
    std::unique_ptr<OptiListener> optitrack;
};

class UserFeedback {
public:
    UserFeedback();

    std::unique_ptr<Buzzer> buzzer;
    std::unique_ptr<LedStrip> leds;
};

class Joints {
public:
    Joints();

    std::unique_ptr<Actuator> wrist_pronosup;
    std::unique_ptr<WristFlexor> wrist_flex;
    std::unique_ptr<Actuator> elbow;
    std::unique_ptr<ShoulderRotator> shoulder;
    std::unique_ptr<TouchBionicsHand> hand;
};

class Components {
public:
    Components();

    UserFeedback user_feedback;
    Sensors sensors;
    Joints joints;

    static const int pin_buzzer = 29;
    static const int pin_demo = 28;
};
}

#endif // SAM_H
