#include "sam.h"
#include "settings.h"

namespace SAM {

Sensors::Sensors()
{
    try {
        myoband = std::make_unique<Myoband>();
        myoband->start();
    } catch (std::exception& e) {
        qCritical() << "Couldn't access the Myoband dongle -" << e.what();
    }

    try {
        arm_imu = std::make_unique<XIMU>("/dev/ximu_white", XIMU::XIMU_LOGLEVEL_NONE, 115200);
    } catch (std::exception& e) {
        qCritical() << "Couldn't access the red IMU -" << e.what();
    }

    try {
        trunk_imu = std::make_unique<XIMU>("/dev/ximu_red", XIMU::XIMU_LOGLEVEL_NONE, 115200);
    } catch (std::exception& e) {
        qCritical() << "Couldn't access the white IMU -" << e.what();
    }

    adc = std::make_unique<Adafruit_ADS1115>("/dev/i2c-1", 0x48);

    Settings s;
    s.beginGroup("Optitrack");
    optitrack = std::make_unique<OptiListener>();
    optitrack->begin(s.value("port", 1511).toInt());
    s.endGroup();
}

UserFeedback::UserFeedback()
{
    buzzer = std::make_unique<Buzzer>(Components::pin_buzzer);

    leds = std::make_unique<LedStrip>();
}

Joints::Joints()
{

    try {
        wrist_flexion = std::make_unique<WristFlexor>();
    } catch (std::exception& e) {
        qCritical() << "Couldn't access the wrist flexor -" << e.what();
    }

    try {
        shoulder_medial_rotation = std::make_unique<ShoulderRotator>();
    } catch (std::exception& e) {
        qCritical() << "Couldn't access the Shoulder rotator -" << e.what();
    }

    try {
        wrist_pronation = std::make_unique<WristRotator>();
    } catch (std::exception& e) {
        qCritical() << "Couldn't access the wrist rotator -" << e.what();
    }

    if (!wrist_pronation) {
        try {
            wrist_pronation = std::make_unique<PronoSupination>();
        } catch (std::exception& e) {
            qCritical() << "Couldn't access the wrist -" << e.what();
        }
    }

    try {
        elbow_flexion = std::make_unique<CustomElbow>();
    } catch (std::exception& e) {
        qCritical() << "Couldn't access the custom elbow -" << e.what();
    }

    if (!elbow_flexion) {
        try {
            elbow_flexion = std::make_unique<OsmerElbow>();
        } catch (std::exception& e) {
            qCritical() << "Couldn't access the elbow -" << e.what();
        }
    }

    try {
        hand = std::make_unique<TouchBionicsHand>();
    } catch (std::exception& e) {
        qCritical() << "Couldn't access the hand -" << e.what();
    }
}

Components::Components() {}

}
