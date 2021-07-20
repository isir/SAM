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
        //auto ng_imu = Components::make_component<NGIMU>("ng_imu", "/dev/ngimu"+std::to_string(cnt), B115200);
        ng_imu = Components::make_component<NGIMU>("ng_imu", "/dev/ngimu"+std::to_string(cnt), B115200);
        if (ng_imu) {
            while(!ng_imu->is_serialnumber_available())
                ng_imu->send_command_serial_number();
            if (ng_imu->get_serialnumber()=="0035F6E2") {
                //red_ngimu = std::move(ng_imu);
                info() << "red_ngimu";
            }
        }
        ++cnt;
    } while (!ng_imu && cnt<5);


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
    buzzer = std::make_unique<Buzzer>(21);
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

   //epos1 = Components::make_component<EPOS>("epos1", "epos1", "/dev/ttyAMA0", 1);
   //epos2 = Components::make_component<EPOS>("epos2", "epos2", "USB1", 1);
}

Components::Components()
    : mosfet_gpio(23, GPIO::DIR_OUTPUT, GPIO::PULL_DOWN)
    , demo_gpio(20, GPIO::DIR_INPUT, GPIO::PULL_UP)
    , adc_gpio(16, GPIO::DIR_INPUT, GPIO::PULL_UP)
    , btn1(19, GPIO::DIR_INPUT, GPIO::PULL_UP)
    , btn2(6, GPIO::DIR_INPUT, GPIO::PULL_UP)
    , btn3(26, GPIO::DIR_INPUT, GPIO::PULL_UP)
{
}
}
