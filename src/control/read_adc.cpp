#include "read_adc.h"
#include "ui/visual/ledstrip.h"
#include <filesystem>
#include <iostream>

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
    _th_low[3] = 3000;
    _th_low[4] = 2000;
    _th_low[5] = 1000;

    _th_high[0] = 3000;
    _th_high[1] = 3000;
    _th_high[2] = 3000;
    _th_high[3] = 3000;
    _th_high[4] = 2000;
    _th_high[5] = 5000;

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

    clock::time_point time2 = clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::microseconds>(time2 - time1).count() << std::endl;
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

    std::string filename("myo");
    std::string suffix;

    int cnt = 0;
    int nbRigidBodies = 0;
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

    _start_time = clock::now();
    return true;
}

void ReadADC::loop(double, clock::time_point time)
{
    double timeWithDelta = (time - _start_time).count();

    _robot->sensors.optitrack->update();
    optitrack_data_t data = _robot->sensors.optitrack->get_last_data();
    //    std::cout << data.nRigidBodies;

    double qBras[4], qTronc[4];
    _robot->sensors.white_imu->get_quat(qBras);
    _robot->sensors.yellow_imu->get_quat(qTronc);

    double beta = _robot->joints.elbow_flexion->pos() * M_PI / 180.;

    std::vector<LedStrip::color> colors(11, LedStrip::white);

    std::string payload;

    /// Old reading
    //    _electrodes[0] = _robot->sensors.adc0->readADC_SingleEnded(2);
    //    _electrodes[1] = _robot->sensors.adc0->readADC_SingleEnded(3);
    //    _electrodes[2] = _robot->sensors.adc2->readADC_SingleEnded(2);
    //    _electrodes[3] = _robot->sensors.adc2->readADC_SingleEnded(0);
    //    _electrodes[4] = _robot->sensors.adc3->readADC_SingleEnded(2);
    //    _electrodes[5] = _robot->sensors.adc3->readADC_SingleEnded(0);

    readAllADC();

    if (_electrodes[2] > _th_high[2] && _electrodes[4] < _th_low[4] && _electrodes[3] < _th_high[3]) {
        _robot->joints.elbow_flexion->set_velocity_safe(25);
    } else if (_electrodes[4] > _th_high[4] && _electrodes[2] < _th_low[2] && _electrodes[3] < _th_high[3]) {
        _robot->joints.elbow_flexion->set_velocity_safe(-25);
    } else {
        _robot->joints.elbow_flexion->set_velocity_safe(0);
    }

    if (_electrodes[3] > _th_high[3] && _electrodes[5] < _th_low[5]) {
        _robot->joints.wrist_pronation->set_velocity_safe(-40);
    } else if (_electrodes[5] > _th_high[5] && _electrodes[3] < _th_low[3]) {
        _robot->joints.wrist_pronation->set_velocity_safe(40);
    } else {
        _robot->joints.wrist_pronation->set_velocity_safe(0);
    }

    if (_electrodes[0] > _th_high[0] && _electrodes[1] < _th_low[1]) {
        _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 1, 1);
    } else if (_electrodes[1] > _th_high[1] && _electrodes[0] < _th_low[0]) {
        _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 2, 1);
    } else {
    }

    for (uint16_t i = 0; i < _n_electrodes; i++) {
        colors[i] = LedStrip::white;

        if (_electrodes[i] > _th_high[i]) {
            colors[i] = LedStrip::red_bright;
        } else if (_electrodes[i] > _th_low[i]) {
            colors[i] = LedStrip::green;
        }
        //        _mqtt.publish("sam/emg/time/" + std::to_string(i), std::to_string(_electrodes[i]));
        std::cout << _electrodes[i] << "\t";
    }
    std::cout << std::endl;
    //    _robot->user_feedback.leds->set(colors);

    std::cout << std::endl;
    //    _robot->user_feedback.leds->set(colors);

    _file << timeWithDelta << ' ' << _electrodes[0] << ' ' << _electrodes[1] << ' ' << _electrodes[2] << ' ' << _electrodes[3] << ' ' << _electrodes[4] << ' ' << _electrodes[5];
    _file << ' ' << qBras[0] << ' ' << qBras[1] << ' ' << qBras[2] << ' ' << qBras[3] << ' ' << qTronc[0] << ' ' << qTronc[1] << ' ' << qTronc[2] << ' ' << qTronc[3];
    _file << ' ' << beta;
    for (unsigned int i = 0; i < data.nRigidBodies; i++) {
        _file << ' ' << data.rigidBodies[i].ID << ' ' << data.rigidBodies[i].bTrackingValid << ' ' << data.rigidBodies[i].fError;
        _file << ' ' << data.rigidBodies[i].qw << ' ' << data.rigidBodies[i].qx << ' ' << data.rigidBodies[i].qy << ' ' << data.rigidBodies[i].qz;
        _file << ' ' << data.rigidBodies[i].x << ' ' << data.rigidBodies[i].y << ' ' << data.rigidBodies[i].z;
    }
    _file << std::endl;

    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()).count() << "ms" << std::endl;
}

void ReadADC::cleanup()
{
    _robot->joints.wrist_pronation->set_velocity_safe(0);
    _robot->joints.elbow_flexion->set_velocity_safe(0);
    _robot->user_feedback.leds->set(LedStrip::none, 10);
    _file.close();
}
