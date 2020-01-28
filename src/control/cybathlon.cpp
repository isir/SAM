#include "cybathlon.h"
#include "utils/check_ptr.h"
#include "utils/log/log.h"
#include <filesystem>
#include "ui/visual/ledstrip.h"
#include <iostream>
#include "algo/myocontrol.h"

Cybathlon::Cybathlon(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("Cybathlon", 0.01)
    , _robot(robot)
    , _lambdaW("lambda wrist", BaseParam::ReadWrite, this, 2)
    , _thresholdW("threshold wrist", BaseParam::ReadWrite, this, 5.)
{
    _menu->set_description("Cybathlon");
    _menu->set_code("cyb");
    _menu->add_item("Tare IMUs", "tare", [this](std::string) { this->tare_IMU(); });
    _menu->add_item("Display Pin data", "pin", [this](std::string) { this->displayPin(); });

    _menu->add_item(_robot->joints.elbow_flexion->menu());
    _menu->add_item(_robot->joints.wrist_pronation->menu());
    _menu->add_item(_robot->joints.hand_quantum->menu());

    _th_low[0] = 3000;
    _th_low[1] = 1500;
    _th_low[2] = 2000;
    _th_low[3] = 2000;
    _th_low[4] = 2000;
    _th_low[5] = 1000;

    _th_high[0] = 3000;
    _th_high[1] = 3000;
    _th_high[2] = 3000;
    _th_high[3] = 2500;
    _th_high[4] = 2000;
    _th_high[5] = 5000;

    for (uint16_t i = 0; i < _n_electrodes; i++) {
        _electrodes[i] = 0;
    }

}

Cybathlon::~Cybathlon()
{
    _robot->joints.elbow_flexion->forward(0);
    _robot->joints.wrist_pronation->forward(0);
    stop_and_join();
}

void Cybathlon::tare_IMU()
{
    if (_robot->sensors.white_imu)
        _robot->sensors.white_imu->send_command_algorithm_init_then_tare();
    if (_robot->sensors.red_imu)
        _robot->sensors.red_imu->send_command_algorithm_init_then_tare();
    if (_robot->sensors.yellow_imu)
        _robot->sensors.yellow_imu->send_command_algorithm_init_then_tare();

    debug("Wait ...");

    std::this_thread::sleep_for(std::chrono::seconds(6));
    _robot->user_feedback.buzzer->makeNoise(Buzzer::TRIPLE_BUZZ);
}

void Cybathlon::displayPin()
{
    int pin_down_value = _robot->btn2;
    int pin_up_value = _robot->btn1;
    debug() << "PinUp: " << pin_up_value;
    debug() << "PinDown: " << pin_down_value;
}

void Cybathlon::readAllADC() //Optimized function to read all 6 electrodes
{
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
    while (!( _robot->sensors.adc0->readRegister(ADS1015_REG_POINTER_CONFIG) >> 15))
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
    while (!( _robot->sensors.adc0->readRegister(ADS1015_REG_POINTER_CONFIG) >> 15))
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
}

bool Cybathlon::setup()
{
    if (_robot->joints.elbow_flexion->is_calibrated() == false)
         _robot->joints.elbow_flexion->calibrate();
    _robot->joints.wrist_pronation->set_encoder_position(0);

    _robot->user_feedback.leds->set(LedStrip::white, 11);

    _cnt = 0.;
    _start_time = clock::now();
    return true;
}

void Cybathlon::loop(double dt, clock::time_point time)
{
    static std::unique_ptr<MyoControl::Classifier> handcontrol;

    static const unsigned int counts_after_mode_change = 15;
    static const unsigned int counts_btn = 2;
    static const unsigned int counts_before_bubble = 2;
    static const unsigned int counts_after_bubble = 2;

    static const MyoControl::EMGThresholds thresholds(5000, 1500, 0, 5000, 1500, 0);

    auto robot = _robot;
    MyoControl::Action co_contraction("Co contraction",
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,1,4); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,1,2); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,2,4); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,2,2); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::CO_CONTRACTION); });
    MyoControl::Action double_contraction("Double contraction",
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,1,4); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,1,2); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,2,4); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,2,2); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::DOUBLE_CONTRACTION); });
    MyoControl::Action triple_contraction("Triple contraction",
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,1,4); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,1,2); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,2,4); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,2,2); },
        [robot]() { robot->joints.hand_quantum->makeContraction(QuantumHand::TRIPLE_CONTRACTION); });
    std::vector<MyoControl::Action> s1 {co_contraction, double_contraction, triple_contraction};

    static bool first = true;
    if (first) {
        handcontrol = std::make_unique<MyoControl::QuantumClassifier>(s1, thresholds, counts_after_mode_change, counts_btn, counts_before_bubble, counts_after_bubble);
        first = false;
    }

    static std::vector<LedStrip::color> colors(11, LedStrip::white);
    static std::string payload;

    readAllADC();

    //Publish EMG values with MQTT
    for (uint16_t i = 0; i < _n_electrodes; i++) {
        _mqtt.publish("sam/emg/time/" + std::to_string(i), std::to_string(_electrodes[i]));
        std::cout << _electrodes[i] << "\t";
    }
    std::cout << std::endl;

    //EMG1 and EMG2 = hand
    static bool btn = 0;
    if (!_robot->btn2) {
        btn = 1;
    } else {
        btn = 0;
    }
    handcontrol->process(_electrodes[0], _electrodes[1], btn);

    static bool elbow_available = 1;
    static bool smoothly_changed_mode = true;

//    if (_robot->btn1) {
//        smoothly_changed_mode = true;
//    }
//    if (smoothly_changed_mode) {
//        if (!_robot->btn1) {
//            elbow_available = !elbow_available;
//            smoothly_changed_mode = false;
//        }
//    }

    if (!_robot->btn1) {
        //EMG3 and EMG5 = elbow
        if (_electrodes[2]>_th_high[2] && _electrodes[4]<_th_low[4] && _electrodes[3]<(_electrodes[2]-1500)) { //EMG3 activation
            _robot->joints.elbow_flexion->set_velocity_safe(25); //Elbow flexion
            colors[2] = LedStrip::red_bright;
        } else if (_electrodes[4]>_th_high[4] && _electrodes[2]<_th_low[2] && _electrodes[3]<_th_low[3] && _electrodes[5]<_th_low[3]) { //EMG5 activation
            _robot->joints.elbow_flexion->set_velocity_safe(-25); //Elbow extension
            colors[4] = LedStrip::red_bright;
        } else {
            _robot->joints.elbow_flexion->set_velocity_safe(0);
        }
    } else {
        _robot->joints.elbow_flexion->set_velocity_safe(0);
    }

    static int previous_value_wrist = 1;

    if(!_robot->btn3) {
        //EMG4 and EMG6 = wrist rotator
        if (_electrodes[3]>_th_high[3] && _electrodes[5]<(_electrodes[3]-500) && _electrodes[4]<(_electrodes[3]-500) && _electrodes[2]<_electrodes[3]) { //EMG4 activation
            _robot->joints.wrist_pronation->set_velocity_safe(-40);
            colors[3] = LedStrip::red_bright;
        } else if (_electrodes[5]>_th_high[5] && _electrodes[3]<_th_high[5] && _electrodes[4]<_th_high[5]) { //EMG6 activation
            _robot->joints.wrist_pronation->set_velocity_safe(40);
            colors[5] = LedStrip::red_bright;
        } else {
            _robot->joints.wrist_pronation->set_velocity_safe(0);
        }
        previous_value_wrist = 0;
    } else {
        //compensation IMU
        if (previous_value_wrist==0) {
            _cnt = 0;
        }
        int init_cnt = 10;

        double qBras[4], qTronc[4], qFA[4];
        if (_robot->sensors.white_imu)
            _robot->sensors.white_imu->get_quat(qBras);
        if (_robot->sensors.red_imu)
            _robot->sensors.red_imu->get_quat(qTronc);
        if (_robot->sensors.yellow_imu)
            _robot->sensors.yellow_imu->get_quat(qFA);

        Eigen::Quaterniond qFA_record;
        qFA_record.w() = qFA[0];
        qFA_record.x() = qFA[1];
        qFA_record.y() = qFA[2];
        qFA_record.z() = qFA[3];

        if (_cnt == 0) {
            _lawimu.initialization();
        } else if (_cnt <= init_cnt) {
            _lawimu.initialPositions(qFA_record, _cnt, init_cnt);
        } else {
            _lawimu.rotationMatrices(qFA_record);
            _lawimu.controlLawWrist(_lambdaW, _thresholdW * M_PI / 180);

            _robot->joints.wrist_pronation->set_velocity_safe(_lawimu.returnWristVel_deg());

            if (_cnt % 50 == 0) {
                _lawimu.displayData();
            }
        }
        ++_cnt;
        previous_value_wrist = 1;
    }

    //LED feedback
    _robot->user_feedback.leds->set(colors);
    for (uint16_t i = 0; i < _n_electrodes; i++) {
        colors[i] = LedStrip::white;
    }

}


void Cybathlon::cleanup()
{
    _robot->joints.wrist_pronation->forward(0);
    _robot->joints.wrist_pronation->set_velocity_safe(0);
    _robot->joints.elbow_flexion->set_velocity_safe(0);
    _robot->user_feedback.leds->set(LedStrip::white, 11);
}
