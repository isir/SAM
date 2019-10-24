#include "sam.h"

namespace SAM {

Sensors::Sensors()
{
    myoband = Components::make_component<Myoband>("myoband");
    if (myoband) {
        myoband->start();
    }

    white_imu = Components::make_component<XIMU>("white_imu", "/dev/ximu_white", XIMU::XIMU_LOGLEVEL_NONE, B115200);
    red_imu = Components::make_component<XIMU>("red_imu", "/dev/ximu_red", XIMU::XIMU_LOGLEVEL_NONE, B115200);
    yellow_imu = Components::make_component<XIMU>("yellow_imu", "/dev/ximu_yellow", XIMU::XIMU_LOGLEVEL_NONE, B115200);

    adc = Components::make_component<Adafruit_ADS1115>("adc", "/dev/i2c-1", 0x48);

    optitrack = Components::make_component<OptiListener>("optitrack");
    if (optitrack) {
        optitrack->begin(1511);
    }
}

UserFeedback::UserFeedback()
{
    buzzer = std::make_unique<Buzzer>(29);
    buzzer->set_prio(90);
    buzzer->set_preferred_cpu(2);

    leds = std::make_unique<LedStrip>();
}

Joints::Joints()
{

    shoulder_medial_rotation = Components::make_component<ShoulderRotator>("shoulder_medial_rotation");
    elbow_flexion = Components::make_component<CustomElbow>("elbow_v2");
    wrist_pronation = Components::make_component<WristRotator>("wrist_pronation_v2");
    wrist_flexion = Components::make_component<WristFlexor>("wrist_flexor");
    hand = Components::make_component<TouchBionicsHand>("touchbionics_hand");

    if (!wrist_pronation) {
        wrist_pronation = Components::make_component<PronoSupination>("wrist_pronation_v1");
    }

    if (!elbow_flexion) {
        elbow_flexion = Components::make_component<OsmerElbow>("elbow_v1");
    }
}

Components::Components()
    : demo_gpio(28, GPIO::DIR_INPUT, GPIO::PULL_UP)
    , btn1(24, GPIO::DIR_INPUT, GPIO::PULL_UP)
    , btn2(22, GPIO::DIR_INPUT, GPIO::PULL_UP)
{
}
}
