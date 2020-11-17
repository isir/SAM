#include "cybathlon.h"
#include "utils/check_ptr.h"
#include "utils/log/log.h"
#include <filesystem>
#include "ui/visual/ledstrip.h"
#include <iostream>
#include "algo/myocontrol.h"

Cybathlon::Cybathlon(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("Cybathlon", 0.04)
    , _robot(robot)
    , _lambdaW("lambda wrist", BaseParam::ReadWrite, this, 2)
    , _thresholdW("threshold wrist", BaseParam::ReadWrite, this, 5.)
{
    if (!check_ptr(_robot->joints.elbow_flexion, _robot->joints.wrist_pronation, _robot->joints.hand_quantum)) {
        throw std::runtime_error("Demo is missing components");
    }

    _menu->set_description("Cybathlon");
    _menu->set_code("cyb");
    _menu->add_item("init", "Initialize NG IMU", [this](std::string) { this->init_IMU(); });
    _menu->add_item("filt", "(De-)activate filtering for EMG signals", [this](std::string) { this->changeFilter(); });

    _menu->add_item(_robot->joints.elbow_flexion->menu());
    _menu->add_item(_robot->joints.wrist_pronation->menu());
    _menu->add_item(_robot->joints.hand_quantum->menu());

    _th[0] = 3500;
    _th[1] = 3500;
    _th[2] = 3000;
    _th[3] = 2500;
    _th[4] = 2000;
    _th[5] = 2500;

    _filter_coef[0] = 0.1881;
    _filter_coef[1] = 0.3428;
    _filter_coef[2] = 0.3428;
    _filter_coef[3] = 0.1881;

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

void Cybathlon::changeFilter()
{
    _filter = !_filter;
    if (_filter) {
        std::cout << "Filtering is now activated" << std::endl;
    } else {
        std::cout << "Filtering is now de-activated" << std::endl;
    }
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

void Cybathlon::processQuantumHand(int emg1, int emg2, int16_t btn_posture) {
    static const unsigned int counts_after_mode_change = 15;
    static const unsigned int counts_btn = 1;
    static const unsigned int counts_before_bubble = 2;
    static const unsigned int counts_after_bubble = 10;
    static bool smoothly_changed_mode = false;
    static bool has_to_check_btn = false;
    static unsigned int mode_changed_counter = 0;
    static unsigned int btn_counter = 0;
    static unsigned int before_bubble_counter = 0;
    static unsigned int after_bubble_counter = 0;
    static unsigned int activated_emg = 1;

    static int16_t posture_th1 = 9000;
    static int16_t posture_th2 = 17000;
    static int16_t posture_th3 = 25000;

    static int posture = 0;

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
                        _robot->user_feedback.buzzer->makeNoise(Buzzer::SHORT_BUZZ);
                        if (btn_posture > posture_th3) {
                            _robot->joints.hand_quantum->makeContraction(QuantumHand::TRIPLE_CONTRACTION);
                            posture = 3;
                        } else if (btn_posture > posture_th2) {
                            _robot->joints.hand_quantum->makeContraction(QuantumHand::DOUBLE_CONTRACTION);
                            posture = 2;
                        } else {
                            _robot->joints.hand_quantum->makeContraction(QuantumHand::CO_CONTRACTION);
                            posture = 1;
                        }
                    } else {
                        btn_counter++;
                    }
                } else {
                    btn_counter = 0;
                    if (emg1 > _th[0] && emg2 < emg1) {
                        ++before_bubble_counter;
                        activated_emg = 1;
                        if (posture==2) {
                            _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,1,2); //fermer main lentement
                        } else if (posture==1){
                            _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,1,4); //fermer main rapidement
                        } else {
                            _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,1,3); //fermer main très rapidement
                        }
                    } else if (emg2 > _th[1] && emg1 < emg2) {
                        ++before_bubble_counter;
                        activated_emg = 2;
                        _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,2,4); //ouvrir main rapidement
                    } else if (emg1 < _th[0] && emg2 < _th[1]) {
                        _robot->joints.hand_quantum->makeContraction(QuantumHand::STOP);
                        activated_emg = 0;
                        before_bubble_counter = 0;
                    }

                    if (before_bubble_counter > counts_before_bubble) {
                        has_to_check_btn = false;
                        after_bubble_counter = 0;
                    }
                }
            } else {
                if (emg1 > _th[0] && emg2 < emg1) {
                    after_bubble_counter = 0;
                    activated_emg = 1;
                    if (posture==2) {
                        _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,1,2); //fermer main lentement
                    } else if (posture==1){
                        _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,1,4); //fermer main rapidement
                    } else {
                        _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,1,3); //fermer main très rapidement
                    }
                } else if (emg2 > _th[1] && emg1 < emg2) {
                    after_bubble_counter = 0;
                    activated_emg = 2;
                    _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION,2,4); //ouvrir main rapidement
                } else if (emg1 < _th[0] && emg2 < _th[1]) {
                    _robot->joints.hand_quantum->makeContraction(QuantumHand::STOP);
                    activated_emg = 0;
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

    _cnt_imu = 0.;

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
    static uint16_t tmp_elec[_n_electrodes][_order_filter+1]={{0}};
    static uint16_t elec_filt[_n_electrodes] = {0};
    static double tmp_elec_filt[_n_electrodes] = {0.};

    readAllADC();

    //Filter EMG signal
    for (int i=0; i<_n_electrodes; i++) {
        for (int j=_order_filter; j>0; j--) {
            tmp_elec[i][j] = tmp_elec[i][j-1];
        }
        tmp_elec[i][0] = _electrodes[i];
        tmp_elec_filt[i] = 0.;
    }
    for (int i=0; i<_n_electrodes; i++) {
        for (int j=0; j<_order_filter+1; j++) {
            tmp_elec_filt[i] = tmp_elec_filt[i] + static_cast<double>(tmp_elec[i][j])*_filter_coef[j];
        }
        elec_filt[i] = static_cast<uint16_t>(tmp_elec_filt[i]);
    }
    if (_filter) {
        for (int i=0; i<_n_electrodes; i++) {
            _electrodes[i] = elec_filt[i];
        }
    }

    //Publish EMG values with MQTT
    for (uint16_t i = 0; i < _n_electrodes; i++) {
        _mqtt.publish("sam/emg/time/" + std::to_string(i), std::to_string(_electrodes[i]));
        std::cout << _electrodes[i] << "\t";
    }
    std::cout << std::endl;

    static int16_t hand_btn;
    hand_btn = _robot->sensors.adc1->readADC_SingleEnded(0);

    //CONTROL
    static int previous_value_wrist = 1;
    static bool elbow_available = 1;
    if(_robot->btn2) { //Mode myo complet
        if (previous_value_wrist==1) {
            _robot->user_feedback.buzzer->makeNoise(Buzzer::SHORT_BUZZ);
        }
        previous_value_wrist = 0;

        //EMG1 and EMG2 = hand
        processQuantumHand(_electrodes[0], _electrodes[1], hand_btn);

        if (_robot->btn1) { //si coude ok
            if (elbow_available==0) {
                _robot->user_feedback.buzzer->makeNoise(Buzzer::SHORT_BUZZ);
            }
            elbow_available = 1;

            if (_electrodes[2]>_th[2] && _electrodes[3]<_electrodes[2]+500 && _electrodes[4]<_electrodes[2] &&_electrodes[5]<_electrodes[2]+500 ) { //EMG3 activation
                _robot->joints.elbow_flexion->set_velocity_safe(25); //Elbow flexion
                _robot->joints.wrist_pronation->set_velocity_safe(0);
                colors[3] = LedStrip::LedStrip::purple;
            } else if (_electrodes[4]>_th[4] && _electrodes[2]<_electrodes[4] && _electrodes[3]<_electrodes[4] && _electrodes[5]<_electrodes[4]) { //EMG5 activation
                _robot->joints.elbow_flexion->set_velocity_safe(-25); //Elbow extension
                _robot->joints.wrist_pronation->set_velocity_safe(0);
                colors[5] = LedStrip::LedStrip::purple;
            } else if (_electrodes[5]>_th[5] && _electrodes[2]<_electrodes[5] && _electrodes[3]<(_electrodes[5]+500) && _electrodes[4]<_electrodes[5]) { //EMG6 activation
                _robot->joints.elbow_flexion->set_velocity_safe(0);
                _robot->joints.wrist_pronation->forward(75); //Wrist
                colors[6] = LedStrip::LedStrip::purple;
            } else if (_electrodes[3]>_th[3] && _electrodes[2]<_electrodes[3] && _electrodes[4]<_electrodes[3] && (_electrodes[5]<_electrodes[3]-500)) { //EMG4 activation
                _robot->joints.elbow_flexion->set_velocity_safe(0);
                _robot->joints.wrist_pronation->backward(75); //Wrist
                colors[4] = LedStrip::LedStrip::purple;
            } else {
                _robot->joints.elbow_flexion->set_velocity_safe(0);
                _robot->joints.wrist_pronation->set_velocity_safe(0);
            }

        } else { //si coude bloqué
            if (elbow_available==1) {
                _robot->user_feedback.buzzer->makeNoise(Buzzer::SHORT_BUZZ);
            }
            elbow_available = 0;
            colors[5] = LedStrip::red_bright;
            _robot->joints.elbow_flexion->set_velocity_safe(0);

            //EMG4 and EMG6 = wrist rotator
            if (_electrodes[5]>_th[5] && _electrodes[2]<_electrodes[5] && (_electrodes[3]<_electrodes[5]+500) && _electrodes[4]<_electrodes[5]) { //EMG6 activation
                _robot->joints.wrist_pronation->forward(75);
                colors[6] = LedStrip::LedStrip::purple;
            } else if (_electrodes[3]>_th[3] && _electrodes[2]<_electrodes[3] && _electrodes[4]<_electrodes[3] && (_electrodes[5]<_electrodes[3]-500)) { //EMG4 activation
                _robot->joints.wrist_pronation->backward(75);
                colors[4] = LedStrip::LedStrip::purple;
            } else {
                _robot->joints.wrist_pronation->set_velocity_safe(0);
            }
        }

        previous_value_wrist = 0;

    } else { //si poignet+main bloqués

        if (previous_value_wrist==0) {
            // _cnt_imu = 0;
            _robot->user_feedback.buzzer->makeNoise(Buzzer::SHORT_BUZZ);
        }
        previous_value_wrist = 1;
        colors[4] = LedStrip::green;

        /*
        //compensation IMU poignet
        int init_cnt_imu = 10;

        double qFA[4];
        if (_robot->sensors.ng_imu) {
            _robot->sensors.ng_imu->get_quat(qFA);
        }

        Eigen::Quaterniond qFA_record;
        qFA_record.w() = qFA[0];
        qFA_record.x() = qFA[1];
        qFA_record.y() = qFA[2];
        qFA_record.z() = qFA[3];

        if (_cnt_imu == 0) {
            _lawimu.initialization();
        } else if (_cnt_imu <= init_cnt_imu) {
            _lawimu.initialPositions(qFA_record, _cnt_imu, init_cnt_imu);
        } else {
            _lawimu.rotationMatrices(qFA_record);
            _lawimu.controlLawWrist(_lambdaW, _thresholdW * M_PI / 180);

            _robot->joints.wrist_pronation->set_velocity_safe(_lawimu.returnWristVel_deg());

            if (_cnt_imu % 50 == 0) {
                _lawimu.displayData();
            }
        }
        ++_cnt_imu; */


        //EMG3 and EMG5 = elbow
        if (_robot->btn1) {
            if (elbow_available==0) {
                _robot->user_feedback.buzzer->makeNoise(Buzzer::SHORT_BUZZ);
            }
            elbow_available = 1;

            if (_electrodes[2]>_th[2] && _electrodes[3]<_electrodes[2]+500 && _electrodes[4]<_electrodes[2] && _electrodes[5]<_electrodes[2]+500 ) { //EMG3 activation
                _robot->joints.elbow_flexion->set_velocity_safe(25); //Elbow flexion
                colors[3] = LedStrip::LedStrip::purple;
            } else if (_electrodes[4]>_th[4] && _electrodes[2]<_electrodes[4] && _electrodes[3]<_electrodes[4] && _electrodes[5]<_electrodes[4]) { //EMG5 activation
                _robot->joints.elbow_flexion->set_velocity_safe(-25); //Elbow extension
                colors[5] = LedStrip::LedStrip::purple;
            } else {
                _robot->joints.elbow_flexion->set_velocity_safe(0);
            }

        } else {
            if (elbow_available==1) {
                _robot->user_feedback.buzzer->makeNoise(Buzzer::SHORT_BUZZ);
            }
            elbow_available = 0;
            colors[5] = LedStrip::red_bright;
            _robot->joints.elbow_flexion->set_velocity_safe(0);
        }
    }

    //LED FEEDBACK
    _robot->user_feedback.leds->set(colors);
    for (uint16_t i = 0; i < 10; i++) {
        colors[i] = LedStrip::white;
    }

    _file << timeWithDelta << "\t" << tmp_elec[0][0] << "\t" << tmp_elec[1][0] << "\t" << tmp_elec[2][0] << "\t" << tmp_elec[3][0] << "\t" << tmp_elec[4][0] << "\t" << tmp_elec[5][0];
    _file << "\t" << hand_btn << "\t" << previous_value_wrist << "\t" << elbow_available;
    _file << "\t" << elec_filt[0] << "\t" << elec_filt[1] << "\t" << elec_filt[2] << "\t" << elec_filt[3] << "\t" << elec_filt[4] << "\t" << elec_filt[5];
    _file << "\t" << _filter;
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
