#include "read_adc.h"
#include "algo/myocontrol.h"
#include "ui/visual/ledstrip.h"
#include <filesystem>
#include <iostream>

// indicate if optitrack is on
#define OPTITRACK 1

ReadADC::ReadADC(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("Read ADC", .025)
    , _robot(robot)
{
    _menu->set_description("Read ADC");
    _menu->set_code("adc");

    _menu->add_item("tare", "Tare IMU", [this](std::string) { this->tareIMU(); });

    _th_low[0] = 3000;
    _th_low[1] = 1500;
    _th_low[2] = 2000;
    _th_low[3] = 2000;
    _th_low[4] = 2000;
    _th_low[5] = 1000;

    _th_high[0] = 3000;
    _th_high[1] = 3000;
    _th_high[2] = 3000;
    _th_high[3] = 3000;
    _th_high[4] = 2000;
    _th_high[5] = 4000;

    for (uint16_t i = 0; i < _n_electrodes; i++) {
        _electrodes[i] = 0;
    }
}

ReadADC::~ReadADC()
{
    stop_and_join();
}

void ReadADC::tareIMU()
{
    if (_robot->sensors.white_imu)
        _robot->sensors.white_imu->send_command_algorithm_init_then_tare();
    if (_robot->sensors.red_imu)
        _robot->sensors.red_imu->send_command_algorithm_init_then_tare();
    if (_robot->sensors.yellow_imu)
        _robot->sensors.yellow_imu->send_command_algorithm_init_then_tare();

    debug("Wait ...");

    usleep(3 * 1000000);
    _robot->user_feedback.buzzer->makeNoise(Buzzer::TRIPLE_BUZZ);
}

void ReadADC::readAllADC() //Optimized function to read all 6 electrodes
{
    clock::time_point time1 = clock::now();

    // Set configuration bits
    uint16_t config_global = ADS1015_REG_CONFIG_CQUE_NONE | // Disable the comparator (default val)
        ADS1015_REG_CONFIG_CLAT_NONLAT | // Non-latching (default val)
        ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
        ADS1015_REG_CONFIG_CMODE_TRAD | // Traditional comparator (default val)
        ADS1015_REG_CONFIG_DR_3300SPS | // 3300 samples per second (ie 860sps pour un ADS1115)
        ADS1015_REG_CONFIG_MODE_SINGLE | // Single-shot mode (default)
        _robot->sensors.adc0->getGain() | //Set PGA/voltage range
        ADS1015_REG_CONFIG_OS_SINGLE; // Set 'start single-conversion' bit

    uint16_t config_c0 = config_global | ADS1015_REG_CONFIG_MUX_SINGLE_0; //for channel 0
    uint16_t config_c1 = config_global | ADS1015_REG_CONFIG_MUX_SINGLE_1; //for channel 1
    uint16_t config_c2 = config_global | ADS1015_REG_CONFIG_MUX_SINGLE_2; //for channel 2
    uint16_t config_c3 = config_global | ADS1015_REG_CONFIG_MUX_SINGLE_3; //for channel 3

    //Write config register to the ADC
    _robot->sensors.adc0->writeRegister(ADS1015_REG_POINTER_CONFIG, config_c2);
    _robot->sensors.adc2->writeRegister(ADS1015_REG_POINTER_CONFIG, config_c2);
    _robot->sensors.adc3->writeRegister(ADS1015_REG_POINTER_CONFIG, config_c2);

    // Wait for the conversion to complete
    while (!(_robot->sensors.adc0->readRegister(ADS1015_REG_POINTER_CONFIG) >> 15))
        ;

    // Read the conversion results
    _electrodes[0] = _robot->sensors.adc0->readRegister(ADS1015_REG_POINTER_CONVERT);
    _electrodes[2] = _robot->sensors.adc2->readRegister(ADS1015_REG_POINTER_CONVERT);
    _electrodes[4] = _robot->sensors.adc3->readRegister(ADS1015_REG_POINTER_CONVERT);

    //Write config register to the ADC
    _robot->sensors.adc0->writeRegister(ADS1015_REG_POINTER_CONFIG, config_c3);
    _robot->sensors.adc2->writeRegister(ADS1015_REG_POINTER_CONFIG, config_c0);
    _robot->sensors.adc3->writeRegister(ADS1015_REG_POINTER_CONFIG, config_c0);

    // Wait for the conversion to complete
    while (!(_robot->sensors.adc0->readRegister(ADS1015_REG_POINTER_CONFIG) >> 15))
        ;

    // Read the conversion results
    _electrodes[1] = _robot->sensors.adc0->readRegister(ADS1015_REG_POINTER_CONVERT);
    _electrodes[3] = _robot->sensors.adc2->readRegister(ADS1015_REG_POINTER_CONVERT);
    _electrodes[5] = _robot->sensors.adc3->readRegister(ADS1015_REG_POINTER_CONVERT);

    for (uint16_t i = 0; i < _n_electrodes; i++) {
        if (_electrodes[i] > 65500) {
            _electrodes[i] = 0;
        }
    }

    //clock::time_point time2 = clock::now();
    //std::cout << std::chrono::duration_cast<std::chrono::microseconds>(time2 - time1).count() << " us" << std::endl;
}

bool ReadADC::setup()
{
    _param_file = std::ifstream("myo_thresholds");
    if (!_param_file.good()) {
        critical() << "Failed to open myo_thresholds file. Using default values instead.";
    } else {
        std::string number_string;
        for (uint16_t i = 0; i < _n_electrodes; i++) {
            std::getline(_param_file, number_string);
            _th_low[i] = std::stoi(number_string);
        }
        for (uint16_t i = 0; i < _n_electrodes; i++) {
            std::getline(_param_file, number_string);
            _th_high[i] = std::stoi(number_string);
        }
    }
    _param_file.close();

    if (_robot->joints.elbow_flexion->is_calibrated() == false)
        _robot->joints.elbow_flexion->calibrate();
    if (_robot->joints.wrist_flexion) {
        _robot->joints.wrist_flexion->calibrate();
    }
    _robot->joints.wrist_pronation->calibrate();

    //    _mqtt.publish("sam/emg/th/1", std::to_string(_th_low[0]));
    //    _mqtt.publish("sam/emg/th/2", std::to_string(_th_high[0]));
    //    _mqtt.subscribe("sam/emg/th/1", Mosquittopp::Client::QoS1)->add_callback(this, [this](Mosquittopp::Message msg) { this->_th_low[0] = std::stoi(msg.payload()); });
    //    _mqtt.subscribe("sam/emg/th/2", Mosquittopp::Client::QoS1)->add_callback(this, [this](Mosquittopp::Message msg) { this->_th_high[0] = std::stoi(msg.payload()); });

    //    _robot->user_feedback.leds->set(LedStrip::white, 10);
    _robot->user_feedback.leds->set(LedStrip::none, 10);

    if (saveData) {
        std::string filename("myo");
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
        }
    }

    _cnt = 0;
    int nbRigidBodies = 0;
    _start_time = clock::now();
    return true;
}

void ReadADC::loop(double, clock::time_point time)
{

    int btnStart;
    if (!_robot->btn3)
        btnStart = 0;
    else
        btnStart = 1;

    int boolBuzz = 0;
    if (_cnt % 100 == 0) {
        // generate random number for buzzer
        boolBuzz = rand() % 2;
        printf("boolBuzz: %d\n", boolBuzz);
        if (boolBuzz == 1)
            _robot->user_feedback.buzzer->makeNoise(Buzzer::STANDARD_BUZZ);
    }

    double timeWithDelta = (time - _start_time).count();
#if OPTITRACK
    _robot->sensors.optitrack->update();
    optitrack_data_t data = _robot->sensors.optitrack->get_last_data();
//    std::cout << data.nRigidBodies;
#endif

    double qBras[4], qTronc[4];
    if (_robot->sensors.white_imu)
        _robot->sensors.white_imu->get_quat(qBras);
    if (_robot->sensors.yellow_imu)
        _robot->sensors.yellow_imu->get_quat(qTronc);

    double beta = _robot->joints.elbow_flexion->pos() * M_PI / 180.;
    double wristAngle = _robot->joints.wrist_pronation->pos() * M_PI / 180.;

    static std::unique_ptr<MyoControl::Classifier> handcontrol;

    static const unsigned int counts_after_mode_change = 15;
    static const unsigned int counts_btn = 2;
    static const unsigned int counts_before_bubble = 2;
    static const unsigned int counts_after_bubble = 2;

    static const MyoControl::EMGThresholds thresholds(5000, 1500, 0, 5000, 1500, 0);

    auto robot = _robot;

    MyoControl::Action co_contraction("Co contraction",
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 1, 4); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 1, 2); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 2, 4); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 2, 2); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::CO_CONTRACTION); });
    MyoControl::Action double_contraction("Double contraction",
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 1, 4); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 1, 2); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 2, 4); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 2, 2); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::DOUBLE_CONTRACTION); });
    MyoControl::Action triple_contraction("Triple contraction",
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 1, 4); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 1, 2); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 2, 4); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 2, 2); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::TRIPLE_CONTRACTION); });

    std::vector<MyoControl::Action> s1{ co_contraction, double_contraction, triple_contraction };

    static bool first = true;
    if (first) {
        handcontrol = std::make_unique<MyoControl::QuantumClassifier>(s1, thresholds, counts_after_mode_change, counts_btn, counts_before_bubble, counts_after_bubble);
        first = false;
    }

    static std::vector<LedStrip::color> colors(11, LedStrip::white);

    static std::string payload;

    /* _electrodes[0] = _robot->sensors.adc0->readADC_SingleEnded(2);
    _electrodes[1] = _robot->sensors.adc0->readADC_SingleEnded(3);
    _electrodes[2] = _robot->sensors.adc2->readADC_SingleEnded(2);
    _electrodes[3] = _robot->sensors.adc2->readADC_SingleEnded(0);
    _electrodes[4] = _robot->sensors.adc3->readADC_SingleEnded(2);
    _electrodes[5] = _robot->sensors.adc3->readADC_SingleEnded(0); */

    readAllADC();

    //EMG3 and EMG5 = elbow
    if (_electrodes[2] > _th_high[2] && _electrodes[4] < _th_low[4] && _electrodes[3] < (_electrodes[2] - 1500)) { //EMG3 activation
        _robot->joints.elbow_flexion->set_velocity_safe(25); //Elbow flexion
        colors[2] = LedStrip::red_bright;
    } else if (_electrodes[4] > _th_high[4] && _electrodes[2] < _th_low[2] && _electrodes[3] < _th_low[3] && _electrodes[5] < _th_low[3]) { //EMG5 activation
        _robot->joints.elbow_flexion->set_velocity_safe(-25); //Elbow extension
        colors[4] = LedStrip::red_bright;
    } else {
        _robot->joints.elbow_flexion->set_velocity_safe(0);
    }

    //EMG4 and EMG6 = wrist rotator
    if (_electrodes[3] > _th_high[3] && _electrodes[5] < (_electrodes[3] - 500) && _electrodes[4] < (_electrodes[3] - 500) && _electrodes[2] < _electrodes[3]) { //EMG4 activation
        _robot->joints.wrist_pronation->set_velocity_safe(-40);
        colors[3] = LedStrip::red_bright;
    } else if (_electrodes[5] > _th_high[5] && _electrodes[3] < _th_high[5] && _electrodes[4] < _th_high[5]) { //EMG6 activation
        _robot->joints.wrist_pronation->set_velocity_safe(40);
        colors[5] = LedStrip::red_bright;
    } else {
        _robot->joints.wrist_pronation->set_velocity_safe(0);
    }

    //EMG1 and EMG2 = hand
    static bool btn = 0;
    if (!_robot->btn1) {
        btn = 1;
    } else {
        btn = 0;
    }
    handcontrol->process(_electrodes[0], _electrodes[1], btn);

    //Publish EMG values with MQTT
    for (uint16_t i = 0; i < _n_electrodes; i++) {
        _mqtt.publish("sam/emg/time/" + std::to_string(i), std::to_string(_electrodes[i]));
        std::cout << _electrodes[i] << "\t";
    }
    std::cout << btn << std::endl;

    //LED feedback
    _robot->user_feedback.leds->set(colors);
    for (uint16_t i = 0; i < _n_electrodes; i++) {
        colors[i] = LedStrip::white;
    }

    std::cout << std::endl;
    //    _robot->user_feedback.leds->set(colors);

    if (saveData) {
        _file << timeWithDelta << ' ' << boolBuzz << ' ' << btnStart << ' ' << _electrodes[0] << ' ' << _electrodes[1] << ' ' << _electrodes[2] << ' ' << _electrodes[3] << ' ' << _electrodes[4] << ' ' << _electrodes[5];
        _file << ' ' << qBras[0] << ' ' << qBras[1] << ' ' << qBras[2] << ' ' << qBras[3] << ' ' << qTronc[0] << ' ' << qTronc[1] << ' ' << qTronc[2] << ' ' << qTronc[3];
        _file << ' ' << beta << ' ' << wristAngle;
#if OPTITRACK
        for (unsigned int i = 0; i < data.nRigidBodies; i++) {
            _file << ' ' << data.rigidBodies[i].ID << ' ' << data.rigidBodies[i].bTrackingValid << ' ' << data.rigidBodies[i].fError;
            _file << ' ' << data.rigidBodies[i].qw << ' ' << data.rigidBodies[i].qx << ' ' << data.rigidBodies[i].qy << ' ' << data.rigidBodies[i].qz;
            _file << ' ' << data.rigidBodies[i].x << ' ' << data.rigidBodies[i].y << ' ' << data.rigidBodies[i].z;
        }
#endif
        _file << std::endl;
    }
    ++_cnt;
    //    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()).count() << "ms" << std::endl;
}

void ReadADC::cleanup()
{
    _robot->joints.wrist_pronation->set_velocity_safe(0);
    _robot->joints.elbow_flexion->set_velocity_safe(0);
    _robot->user_feedback.leds->set(LedStrip::white, 11);
    if (saveData)
        _file.close();
}
