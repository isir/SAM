#ifndef SAM_H
#define SAM_H

#include "components/external/myoband/myoband.h"
#include "components/external/optitrack_listener.h"
#include "components/external/ximu.h"
#include "components/internal/actuators/actuator.h"
#include "components/internal/actuators/custom_elbow.h"
#include "components/internal/actuators/osmer_elbow.h"
#include "components/internal/actuators/pronosupination.h"
#include "components/internal/actuators/shoulder_rotator.h"
#include "components/internal/actuators/wrist_flexor.h"
#include "components/internal/actuators/wrist_rotator.h"
#include "components/internal/adafruit_ads1115.h"
#include "components/internal/touch_bionics/touch_bionics_hand.h"
#include "ui/sound/buzzer.h"
#include "ui/visual/ledstrip.h"
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

    std::unique_ptr<Actuator> wrist_pronation;
    std::unique_ptr<WristFlexor> wrist_flexion;
    std::unique_ptr<Actuator> elbow_flexion;
    std::unique_ptr<ShoulderRotator> shoulder_medial_rotation;
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
