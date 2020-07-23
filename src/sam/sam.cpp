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

    int cnt = 0;
    do {
        ng_imu = Components::make_component<NGIMU>("ng_imu", "/dev/ngimu"+std::to_string(cnt), B115200);
        ++cnt;
    } while (!ng_imu);


    adc0 = Components::make_component<Adafruit_ADS1115>("adc0", "/dev/i2c-1", 0x48);
    adc1 = Components::make_component<Adafruit_ADS1115>("adc1", "/dev/i2c-1", 0x49);
    adc2 = Components::make_component<Adafruit_ADS1115>("adc2", "/dev/i2c-1", 0x4B);
    adc3 = Components::make_component<Adafruit_ADS1115>("adc3", "/dev/i2c-1", 0x4A);

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
    if (!hand) {
        hand_quantum = Components::make_component<QuantumHand>("quantum_hand");
    }

    if (!wrist_pronation) {
        wrist_pronation = Components::make_component<PronoSupination>("wrist_pronation_v1");
        if (!wrist_pronation) {
            wrist_pronation = Components::make_component<WristCybathlon>("wrist_pronation_cybathlon");
        }
    }

    if (!elbow_flexion) {
        elbow_flexion = Components::make_component<OsmerElbow>("elbow_v1");
        if (!elbow_flexion) {
            elbow_flexion = Components::make_component<ElbowCybathlon>("elbow_cybathlon");
        }
    }
}

Components::Components()
    : mosfet_gpio(4,GPIO::DIR_OUTPUT, GPIO::PULL_DOWN)
    , demo_gpio(28, GPIO::DIR_INPUT, GPIO::PULL_UP)
    , adc_gpio(27, GPIO::DIR_INPUT, GPIO::PULL_UP)
    , btn1(24, GPIO::DIR_INPUT, GPIO::PULL_UP)
    , btn2(22, GPIO::DIR_INPUT, GPIO::PULL_UP)
    , btn3(25, GPIO::DIR_INPUT, GPIO::PULL_UP)
{
}
}
