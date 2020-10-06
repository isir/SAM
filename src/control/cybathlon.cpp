#include "cybathlon.h"
#include "utils/check_ptr.h"
#include "utils/log/log.h"
#include <filesystem>
#include "ui/visual/ledstrip.h"
#include <iostream>
#include "algo/myocontrol.h"

Cybathlon::Cybathlon(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("Cybathlon", 0.025)
    , _robot(robot)
    , _lambdaW("lambda wrist", BaseParam::ReadWrite, this, 2)
    , _thresholdW("threshold wrist", BaseParam::ReadWrite, this, 5.)
{
    if (!check_ptr(_robot->joints.elbow_flexion, _robot->joints.wrist_pronation, _robot->joints.hand_quantum)) {
        throw std::runtime_error("Demo is missing components");
    }

    _menu->set_description("Cybathlon");
    _menu->set_code("cyb");
    _menu->add_item("tare", "Tare IMUs", [this](std::string) { this->tare_IMU(); });
    _menu->add_item("init", "Initialize NG IMUs", [this](std::string) { this->init_IMU(); });

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
    _th_high[5] = 2500;

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

void Cybathlon::init_IMU()
{
    if (_robot->sensors.ng_imu)
        _robot->sensors.ng_imu->send_command_algorithm_init();

    debug("Wait ...");

    std::this_thread::sleep_for(std::chrono::seconds(6));
    _robot->user_feedback.buzzer->makeNoise(Buzzer::TRIPLE_BUZZ);
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
        if (_electrodes[i] > 60000) {
            _electrodes[i] = 0;
        }
    }
}

void Cybathlon::processQuantumHand(int emg1, int emg2, uint16_t btn_posture) {
    static const unsigned int counts_after_mode_change = 15;
    static const unsigned int counts_btn = 2;
    static const unsigned int counts_before_bubble = 2;
    static const unsigned int counts_after_bubble = 10;
    static bool smoothly_changed_mode = false;
    static bool has_to_check_btn = false;
    static unsigned int mode_changed_counter = 0;
    static unsigned int btn_counter = 0;
    static unsigned int before_bubble_counter = 0;
    static unsigned int after_bubble_counter = 0;
    static unsigned int activated_emg = 1;

    static uint16_t posture_th1 = 9000;
    static uint16_t posture_th2 = 17000;
    static uint16_t posture_th3 = 25000;

    static const MyoControl::EMGThresholds thresholds(4000, 2200, 0, 4000, 1300, 0);


    if (mode_changed_counter > counts_after_mode_change) { //si on a laissé assez de temps après le changement de posture
        if (btn_posture < posture_th1) { //si on a bien relaché le bouton
            smoothly_changed_mode = true;
        }
        if (smoothly_changed_mode) { //si le bouton a bien été relaché depuis le dernier changement de mode
            if (has_to_check_btn) { //s'il s'est écoulé suffisamment de temps depuis la dernière contraction
                if (btn_posture > posture_th1) { //si appui sur le bouton
                    if (btn_counter > counts_btn) { //si le bouton est maintenu assez longtemps
                        btn_counter = 0;
                        mode_changed_counter = 0;
                        smoothly_changed_mode = false;
                        if (btn_posture > posture_th3) {
                            _robot->joints.hand_quantum->makeContraction(QuantumHand::TRIPLE_CONTRACTION);
                        } else if (btn_posture > posture_th2) {
                            _robot->joints.hand_quantum->makeContraction(QuantumHand::DOUBLE_CONTRACTION);
                        } else {
                            _robot->joints.hand_quantum->makeContraction(QuantumHand::CO_CONTRACTION);
                        }
                    } else {
                        btn_counter++;
                    }
                }
                else {
                    btn_counter = 0;
                    if (emg1 > thresholds.forward_upper) {
                        ++before_bubble_counter;
                        activated_emg = 1;
                        _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,1,2); //fermer main lentement
//                    } else if (emg1 > _thresholds.backward_lower) {
//                        ++before_bubble_counter;
//                        activated_emg = 1;
//                       _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,1,2); //fermer main lentement
                    } else if (emg2 > thresholds.backward_upper) {
                        ++before_bubble_counter;
                        activated_emg = 2;
                        _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,2,4); //ouvrir main rapidement
//                    } else if (emg2 > thresholds.backward_lower) {
//                        ++before_bubble_counter;
//                        activated_emg = 2;
//                        _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,2,2); //ouvrir main lentement
                    } else {
                        before_bubble_counter = 0;
                    }

                    if (before_bubble_counter > counts_before_bubble) {
                        has_to_check_btn = false;
                        after_bubble_counter = 0;
                    }
                }
            } else {
                if (emg1 > thresholds.forward_upper && activated_emg == 1) {
                    after_bubble_counter = 0;
                    // _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,1,4); //fermer main rapidement
                    _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,1,2); //fermer main lentement
//                } else if (emg1 > _thresholds.backward_lower && activated_emg == 1) {
//                    after_bubble_counter = 0;
//                    _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,1,2); //fermer main lentement
                } else if (emg2 > thresholds.backward_upper && activated_emg == 2) {
                    after_bubble_counter = 0;
                    _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,2,4); //ouvrir main rapidement
//                } else if (emg2 > thresholds.backward_lower && activated_emg == 2) {
//                    after_bubble_counter = 0;
//                    _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,2,2); //ouvrir main lentement
                } else {
                    ++after_bubble_counter;
                }

                if (after_bubble_counter > counts_after_bubble) {
                    has_to_check_btn = true;
                }
            }
        }
    } else
        mode_changed_counter++;
}

bool Cybathlon::setup()
{
    if (_robot->joints.elbow_flexion->is_calibrated() == false)
         _robot->joints.elbow_flexion->calibrate();
    _robot->joints.wrist_pronation->set_encoder_position(0);

    _robot->user_feedback.leds->set(LedStrip::white, 11);

    _cnt = 0.;

    // Record myo data in txt file
    std::string filename("myo_cybathlon");
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

    _start_time = clock::now();
    return true;
}

void Cybathlon::loop(double dt, clock::time_point time)
{
    double timeWithDelta = std::chrono::duration_cast<std::chrono::microseconds>(time - _start_time).count();

    static std::vector<LedStrip::color> colors(11, LedStrip::white);
    static std::string payload;

    readAllADC();

    //EMG1 and EMG2 = hand
    static uint16_t hand_btn;
    hand_btn = _robot->sensors.adc1->readADC_SingleEnded(0);
    processQuantumHand(_electrodes[0], _electrodes[1], hand_btn);

    //Publish EMG values with MQTT
    for (uint16_t i = 0; i < _n_electrodes; i++) {
        _mqtt.publish("sam/emg/time/" + std::to_string(i), std::to_string(_electrodes[i]));
        std::cout << _electrodes[i] << "\t";
    }
    std::cout << hand_btn << std::endl;

    // ELBOW CONTROL
    static bool elbow_available = 1;
    if (!_robot->btn1) {
        //EMG3 and EMG5 = elbow
        if (_electrodes[2]>_th_high[2] && _electrodes[5]<_electrodes[2]+500 && _electrodes[3]<_electrodes[2]+500) { //EMG3 activation
            _robot->joints.elbow_flexion->set_velocity_safe(25); //Elbow flexion
            colors[2] = LedStrip::red_bright;
        } else if (_electrodes[4]>_th_high[4] && _electrodes[2]<_th_low[2] && _electrodes[3]<_th_low[3] && _electrodes[5]<_th_low[3]) { //EMG5 activation
            _robot->joints.elbow_flexion->set_velocity_safe(-25); //Elbow extension
            colors[4] = LedStrip::red_bright;
        } else {
            _robot->joints.elbow_flexion->set_velocity_safe(0);
        }
        elbow_available = 1;
    } else {
        _robot->joints.elbow_flexion->set_velocity_safe(0);
        elbow_available = 0;
    }

    // WRIST CONTROL
    static int previous_value_wrist = 1;
    if(!_robot->btn2) { //Mode myo
        //EMG4 and EMG6 = wrist rotator
        if (_electrodes[3]>_th_high[3] && _electrodes[5]<(_electrodes[3]-500) && _electrodes[4]<(_electrodes[3]-500) && _electrodes[2]<_electrodes[3]) { //EMG4 activation
            _robot->joints.wrist_pronation->set_velocity_safe(-120);
            colors[3] = LedStrip::red_bright;
        } else if (_electrodes[5]>_th_high[5] && _electrodes[3]<_electrodes[5] && _electrodes[4]<_electrodes[5]) { //EMG6 activation
            _robot->joints.wrist_pronation->set_velocity_safe(120);
            colors[5] = LedStrip::red_bright;
        } else {
            _robot->joints.wrist_pronation->set_velocity_safe(0);
        }
        previous_value_wrist = 0;
    } else { //compensation IMU
        if (previous_value_wrist==0) {
            _cnt = 0;
        }
        int init_cnt = 10;

        double qBras[4], qTronc[4], qFA[4];
        if (_robot->sensors.white_imu)
            _robot->sensors.white_imu->get_quat(qBras);
        if (_robot->sensors.red_imu)
            _robot->sensors.red_imu->get_quat(qTronc);
        if (_robot->sensors.yellow_imu) {
            _robot->sensors.yellow_imu->get_quat(qFA);
        } else if (_robot->sensors.ng_imu) {
            _robot->sensors.ng_imu->get_quat(qFA);
        }

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

    //LED FEEDBACK
    _robot->user_feedback.leds->set(colors);
    for (uint16_t i = 0; i < _n_electrodes; i++) {
        colors[i] = LedStrip::white;
    }

    _file << timeWithDelta << "\t" << _electrodes[0] << "\t" << _electrodes[1] << "\t" << _electrodes[2] << "\t" << _electrodes[3] << "\t" << _electrodes[4] << "\t" << _electrodes[5];
    _file << "\t" << hand_btn << "\t" << previous_value_wrist << "\t" << elbow_available;
    _file << std::endl;

}


void Cybathlon::cleanup()
{
    _robot->joints.wrist_pronation->forward(0);
    _robot->joints.wrist_pronation->set_velocity_safe(0);
    _robot->joints.elbow_flexion->set_velocity_safe(0);
    _robot->user_feedback.leds->set(LedStrip::white, 11);
    _file.close();
}
