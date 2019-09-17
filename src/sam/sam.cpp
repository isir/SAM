#include "sam.h"

namespace SAM {

Sensors::Sensors()
{
    myoband = Components::make_component<Myoband>("myoband");
    if (myoband) {
        myoband->start();
    }

    arm_imu = Components::make_component<XIMU>("white_imu", "/dev/ximu_white", XIMU::XIMU_LOGLEVEL_NONE, 115200);
    trunk_imu = Components::make_component<XIMU>("red_imu", "/dev/ximu_red", XIMU::XIMU_LOGLEVEL_NONE, 115200);
    fa_imu = Components::make_component<XIMU>("yellow_imu", "/dev/ximu_yellow", XIMU::XIMU_LOGLEVEL_NONE, 115200);

    adc = Components::make_component<Adafruit_ADS1115>("adc", "/dev/i2c-1", 0x48);

    optitrack = Components::make_component<OptiListener>("optitrack");
    if (optitrack) {
        optitrack->begin(1511);
    }
}

UserFeedback::UserFeedback()
{
    buzzer = std::make_unique<Buzzer>(Components::pin_buzzer);
    buzzer->set_prio(90);
    buzzer->set_preferred_cpu(2);

    leds = std::make_unique<LedStrip>();
}

Joints::Joints()
{

    try {
        wrist_flexion = std::make_unique<WristFlexor>();
    } catch (std::exception& e) {
        critical() << "Couldn't access the wrist flexor - " << e.what();
    }

    try {
        shoulder_medial_rotation = std::make_unique<ShoulderRotator>();
    } catch (std::exception& e) {
        critical() << "Couldn't access the Shoulder rotator - " << e.what();
    }

    try {
        wrist_pronation = std::make_unique<WristRotator>();
    } catch (std::exception& e) {
        critical() << "Couldn't access the wrist rotator - " << e.what();
    }

    if (!wrist_pronation) {
        try {
            wrist_pronation = std::make_unique<PronoSupination>();
        } catch (std::exception& e) {
            critical() << "Couldn't access the wrist - " << e.what();
        }
    }

    try {
        elbow_flexion = std::make_unique<CustomElbow>();
    } catch (std::exception& e) {
        critical() << "Couldn't access the custom elbow - " << e.what();
    }

    if (!elbow_flexion) {
        try {
            elbow_flexion = std::make_unique<OsmerElbow>();
        } catch (std::exception& e) {
            critical() << "Couldn't access the elbow - " << e.what();
        }
    }

    try {
        hand = std::make_unique<TouchBionicsHand>();
    } catch (std::exception& e) {
        critical() << "Couldn't access the hand - " << e.what();
    }
}

Components::Components() {}
}
