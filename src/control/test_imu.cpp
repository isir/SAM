#include "test_imu.h"
#include "algo/myocontrol.h"
#include "utils/check_ptr.h"
#include <filesystem>
#include "ui/visual/ledstrip.h"
#include "utils/log/log.h"
#include <iostream>

TestIMU::TestIMU(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("Test IMU", 0.025)
    , _robot(robot)
{
    if (!check_ptr(_robot->sensors.white_imu, _robot->sensors.ng_imu, _robot->joints.elbow_flexion, _robot->joints.wrist_pronation, _robot->joints.hand_quantum)) {
        throw std::runtime_error("TestIMU is missing components");
    }
    _menu->set_description("Test IMU");
    _menu->set_code("testimu");
    _menu->add_item("tare", "Tare IMUs", [this](std::string) { this->tare_IMU(); });
    _menu->add_item(_robot->joints.elbow_flexion->menu());
    _menu->add_item(_robot->joints.wrist_pronation->menu());
    _menu->add_item(_robot->joints.hand_quantum->menu());

}

TestIMU::~TestIMU()
{
    stop_and_join();
}

void TestIMU::tare_IMU()
{
    if (_robot->sensors.white_imu)
        _robot->sensors.white_imu->send_command_algorithm_init_then_tare();
    if (_robot->sensors.ng_imu)
        _robot->sensors.ng_imu->send_command_algorithm_init();

    debug("Wait ...");

    usleep(6 * 1000000);
    _robot->user_feedback.buzzer->makeNoise(Buzzer::TRIPLE_BUZZ);
}

bool TestIMU::setup()
{
    if (_robot->joints.elbow_flexion->is_calibrated() == false)
         _robot->joints.elbow_flexion->calibrate();
    _robot->joints.wrist_pronation->set_encoder_position(0);

    _robot->user_feedback.leds->set(LedStrip::white, 11);

    // Record IMU data in txt file
    std::string filename("comparaison_IMU");
    std::string suffix;
    int cnt = 0;
    std::string extension(".txt");
    do {
        ++cnt;
        suffix = "_" + std::to_string(cnt);
    } while (std::filesystem::exists(filename + suffix + extension));
    _file = std::ofstream(filename + suffix + extension);
    if (!_file.good()) {
        critical() << "Failed to open" << (filename + suffix + extension);
        return false;
    } else {
       /// WRITE FILE HEADERS
        _file << "quatximu.w \t quatximu.x \t quatximu.y \t quatximu.z \t";
        _file << "gyroximu.x \t gyroximu.y \t gyroximu.z \t";
        _file << "accximu.x \t accximu.y \t accximu.z \t";
        _file << "quatngimu.w \t quatngimu.x \t quatngimu.y \t quatngimu.z \t";
        _file << "gyrongimu.x \t gyrongimu.y \t gyrongimu.z \t";
        _file << "accngimu.x \t accngimu.y \t accngimu.z \t";
        _file << std::endl;
    }

    _start_time = clock::now();
    return true;
}

void TestIMU::loop(double dt, clock::time_point time)
{
    //Myo demo control
    static std::unique_ptr<MyoControl::Classifier> myocontrol;
    static const unsigned int counts_after_mode_change = 10;
    static const unsigned int counts_cocontraction = 5;
    static const unsigned int counts_before_bubble = 5;
    static const unsigned int counts_after_bubble = 5;
    static const MyoControl::EMGThresholds thresholds(15, 8, 15, 15, 8, 15);
    auto robot = _robot;
    MyoControl::Action elbow(
        "Elbow", [robot]() { robot->joints.elbow_flexion->set_velocity_safe(-25); }, [robot]() { robot->joints.elbow_flexion->set_velocity_safe(25); }, [robot]() { robot->joints.elbow_flexion->set_velocity_safe(0); });
    MyoControl::Action wrist_pronosup(
        "Wrist rotation", [robot]() { robot->joints.wrist_pronation->set_velocity_safe(40); }, [robot]() { robot->joints.wrist_pronation->set_velocity_safe(-40); }, [robot]() { robot->joints.wrist_pronation->set_velocity_safe(0); });
    MyoControl::Action hand(
        "Hand", [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,1,4); }, [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,2,4); }, [robot]() { robot->joints.wrist_pronation->set_velocity_safe(0); });
    static LedStrip::color current_color = LedStrip::none;
    std::vector<MyoControl::Action> s1 { hand, wrist_pronosup, elbow };

    int emg[2];
    static bool first = true;
    if (first) {
        current_color = LedStrip::green;
        myocontrol = std::make_unique<MyoControl::BubbleCocoClassifier>(s1, thresholds, counts_after_mode_change, counts_cocontraction, counts_before_bubble, counts_after_bubble);
        first = false;
    }
    emg[0] = 0;
    emg[1] = 0;
    if (!_robot->btn1) {
        emg[0] = 80;
    }
    if (!_robot->btn2) {
        emg[1] = 80;
    }
    myocontrol->process(emg[0], emg[1]);

    std::vector<LedStrip::color> colors(11, current_color);
    switch (myocontrol->current_index()) {
    case 0:
        colors[4] = LedStrip::color(80, 30, 0, 1);
        break;
    case 1:
        colors[4] = LedStrip::color(0, 50, 50, 1);
        break;
    case 2:
        colors[4] = LedStrip::color(50, 0, 50, 1);
        break;
    case 3:
        colors[4] = LedStrip::white;
        break;
    default:
        break;
    }
    _robot->user_feedback.leds->set(colors);


    //Get IMU data
    double quatx[4], quatng[4], gyrox[3], accx[3], magx[3], sensorsng[10];
    _robot->sensors.white_imu->get_quat(quatx);
    _robot->sensors.white_imu->get_cal(gyrox, accx, magx);
    _robot->sensors.ng_imu->get_quat(quatng);
    _robot->sensors.ng_imu->get_sensors(sensorsng);

    //Publish IMU values with MQTT
    static bool publish = 1;
    publish = !publish;
    if (publish) {
    for (uint16_t i = 0; i < 3; i++) {
        _mqtt.publish("sam/emg/time/" + std::to_string(i), std::to_string(quatx[i]));
    }
    for (uint16_t i = 0; i < 3; i++) {
        _mqtt.publish("sam/emg/time/" + std::to_string(i+3), std::to_string(quatng[i]));
    }
    }


    _file << quatx[0] << "\t" << quatx[1] << "\t" << quatx[2] << "\t" << quatx[3] << "\t" << gyrox[0] << "\t" << gyrox[1] << "\t" << gyrox[2] << "\t" << accx[0] << "\t" << accx[1] << "\t" << accx[2] << "\t";
    _file << quatng[0] << "\t" << quatng[1] << "\t" << quatng[2] << "\t" << quatng[3] << "\t" << sensorsng[0] << "\t" << sensorsng[1] << "\t" << sensorsng[2] << "\t" << sensorsng[3] << "\t" << sensorsng[4] << "\t" << sensorsng[5] ;
    _file << std::endl;
}

void TestIMU::cleanup()
{
    _file.close();
    _robot->joints.wrist_pronation->set_velocity_safe(0);
    _robot->joints.elbow_flexion->set_velocity_safe(0);
    _robot->user_feedback.leds->set(LedStrip::white, 11);
}
