#ifndef SAM_H
#define SAM_H

#include "components/external/myoband/myoband.h"
#include "components/external/optitrack/optitrack_listener.h"
#include "components/external/ximu/ximu.h"
#include "components/external/ngimu/ngimu.h"
#include "components/internal/actuators/actuator.h"
#include "components/internal/actuators/custom_elbow.h"
#include "components/internal/actuators/elbow_cybathlon.h"
#include "components/internal/actuators/osmer_elbow.h"
#include "components/internal/actuators/pronosupination.h"
#include "components/internal/actuators/shoulder_rotator.h"
#include "components/internal/actuators/wrist_cybathlon.h"
#include "components/internal/actuators/wrist_flexor.h"
#include "components/internal/actuators/wrist_rotator.h"
#include "components/internal/actuators/epos/epos.h"
#include "components/internal/adc/adafruit_ads1115.h"
#include "components/internal/gpio/gpio.h"
#include "components/internal/hand/touch_bionics_hand.h"
#include "components/internal/hand/quantum_hand.h"
#include "ui/sound/buzzer.h"
#include "ui/visual/ledstrip.h"
#include "utils/log/log.h"
#include "utils/named_object.h"
#include "ux/mosquittopp/client.h"
#include <memory>

namespace SAM {

template <typename U, typename... Ts>
std::unique_ptr<U> make_generic(std::string type, std::string name, Ts... args)
{
    Mosquittopp::Client mqtt;
    mqtt.connect(MOSQUITTO_SERVER_IP, MOSQUITTO_SERVER_PORT);

    std::unique_ptr<U> p;
    try {
        p = std::make_unique<U>(args...);
        mqtt.publish(NamedObject::base_name + "/" + type + "/" + name, std::string("1"), Mosquittopp::Client::QoS1, true);
        info() << "Created this component: " << name;
    } catch (std::exception& e) {
        critical() << "Couldn't create this component: " << name << " (" << e.what() << ")";
        mqtt.publish(NamedObject::base_name + "/" + type + "/" + name, std::string("0"), Mosquittopp::Client::QoS1, true);
    }
    return p;
}

class Sensors {
public:
    Sensors();

    std::unique_ptr<Myoband> myoband;
    std::unique_ptr<XIMU> white_imu;
    std::unique_ptr<XIMU> red_imu;
    std::unique_ptr<XIMU> yellow_imu;
    std::unique_ptr<NGIMU> ng_imu;
    std::unique_ptr<NGIMU> red_ngimu;
    std::unique_ptr<Adafruit_ADS1115> adc0;
    std::unique_ptr<Adafruit_ADS1115> adc1;
    std::unique_ptr<Adafruit_ADS1115> adc2;
    std::unique_ptr<Adafruit_ADS1115> adc3;
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
    std::unique_ptr<QuantumHand> hand_quantum;

    std::unique_ptr<EPOS> epos1;
    std::unique_ptr<EPOS> epos2;
};

class Components {
public:
    Components();

    UserFeedback user_feedback;
    Sensors sensors;
    Joints joints;

    GPIO mosfet_gpio;
    GPIO demo_gpio;
    GPIO adc_gpio;
    GPIO btn1;
    GPIO btn2;
    GPIO btn3;

    template <typename U, typename... Ts>
    inline static std::unique_ptr<U> make_component(std::string name, Ts... args)
    {
        return make_generic<U>("component", name, args...);
    }
};
}

#endif // SAM_H
