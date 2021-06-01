#include "myo_2electrodes.h"
#include "algo/myocontrol.h"
#include "ui/visual/ledstrip.h"
#include "utils/check_ptr.h"
#include "utils/log/log.h"
#include <filesystem>

// indicate if optitrack is on
#define OPTITRACK 1

myo_2electrodes::myo_2electrodes(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("myo_2electrodes", .025)
    , _robot(robot)
    , _forward_upper("F upper", BaseParam::ReadWrite, this, 3500)
    , _forward_lower("F lower", BaseParam::ReadWrite, this, 3500)
    , _backward_upper("B upper", BaseParam::ReadWrite, this, 3500)
    , _backward_lower("B lower", BaseParam::ReadWrite, this, 3500)
{
    if (!check_ptr(_robot->joints.elbow_flexion, _robot->joints.wrist_pronation)) {
        throw std::runtime_error("myo_2electrodes is missing components");
    }

    _menu->set_description("myo_2electrodes");
    _menu->set_code("myo2");

    _menu->add_item(_robot->joints.elbow_flexion->menu());
    _menu->add_item(_robot->joints.wrist_pronation->menu());
    _menu->add_item("tareAll", "Tare all IMUs", [this](std::string) { this->tare_allIMU(); });
    _menu->add_item("tareWhite", "Tare white IMU (trunk)", [this](std::string) { this->tare_whiteIMU(); });
    _menu->add_item("tareYellow", "Tare yellow IMU (hip)", [this](std::string) { this->tare_yellowIMU(); });
    _menu->add_item("tareRed", "Tare red IMU (hand)", [this](std::string) { this->tare_redIMU(); });
    _menu->add_item("calib", "Calibrations", [this](std::string) { this->calibrations(); });
    _menu->add_item("elbow90", "Flex elbow at 90 deg", [this](std::string) { this->elbowTo90(); });

    for (uint16_t i = 0; i < _n_electrodes; i++) {
        _electrodes[i] = 0;
    }
}

myo_2electrodes::~myo_2electrodes()
{
    stop_and_join();
}

void myo_2electrodes::tare_allIMU()
{
    if (!_robot->sensors.red_imu || !_robot->sensors.yellow_imu) {
        std::cout << "An IMU is missing." << std::endl;
        critical() << "An IMU is missing.";
        _robot->user_feedback.buzzer->makeNoise(Buzzer::ERROR_BUZZ);
    } else {

        if (_robot->sensors.white_imu)
            _robot->sensors.white_imu->send_command_algorithm_init_then_tare();
        if (_robot->sensors.yellow_imu)
            _robot->sensors.yellow_imu->send_command_algorithm_init_then_tare();
        if (_robot->sensors.red_imu)
            _robot->sensors.red_imu->send_command_algorithm_init_then_tare();

        debug("Wait ...");

        usleep(6 * 1000000);

        double qWhite[4], qYellow[4], qRed[4];
        if (_robot->sensors.white_imu) {
            _robot->sensors.white_imu->get_quat(qWhite);
            debug() << "qWhite: " << qWhite[0] << "; " << qWhite[1] << "; " << qWhite[2] << "; " << qWhite[3];
        }
        if (_robot->sensors.yellow_imu) {
            _robot->sensors.yellow_imu->get_quat(qYellow);
            debug() << "qyellow: " << qYellow[0] << "; " << qYellow[1] << "; " << qYellow[2] << "; " << qYellow[3];
        }
        if (_robot->sensors.red_imu) {
            _robot->sensors.red_imu->get_quat(qRed);
            debug() << "qred: " << qRed[0] << "; " << qRed[1] << "; " << qRed[2] << "; " << qRed[3];
        }

        if (qRed[0] < 5E-5 || qYellow[0] < 5E-5) {
            std::cout << "An IMU was not correctly tared." << std::endl;
            critical() << "An IMU was not correctly tared.";
            _robot->user_feedback.buzzer->makeNoise(Buzzer::ERROR_BUZZ);
        } else {
            _robot->user_feedback.buzzer->makeNoise(Buzzer::TRIPLE_BUZZ);
        }
    }
}

void myo_2electrodes::tare_whiteIMU()
{
    if (!_robot->sensors.white_imu) {
        std::cout << "White IMU is missing." << std::endl;
        critical() << "White IMU is missing.";
        _robot->user_feedback.buzzer->makeNoise(Buzzer::ERROR_BUZZ);
    } else {

        _robot->sensors.white_imu->send_command_algorithm_init_then_tare();

        debug("Wait ...");

        usleep(6 * 1000000);

        double qWhite[4];
        _robot->sensors.white_imu->get_quat(qWhite);
        debug() << "qWhite: " << qWhite[0] << "; " << qWhite[1] << "; " << qWhite[2] << "; " << qWhite[3];

        if (qWhite[0] < 5E-5) {
            std::cout << "White IMU was not correctly tared." << std::endl;
            critical() << "White IMU was not correctly tared.";
            _robot->user_feedback.buzzer->makeNoise(Buzzer::ERROR_BUZZ);
        } else {
            _robot->user_feedback.buzzer->makeNoise(Buzzer::TRIPLE_BUZZ);
        }
    }
}

void myo_2electrodes::tare_yellowIMU()
{
    if (!_robot->sensors.yellow_imu) {
        std::cout << "Yellow IMU is missing." << std::endl;
        critical() << "Yellow IMU is missing.";
        _robot->user_feedback.buzzer->makeNoise(Buzzer::ERROR_BUZZ);
    } else {
        _robot->sensors.yellow_imu->send_command_algorithm_init_then_tare();

        debug("Wait ...");

        usleep(6 * 1000000);

        double qYellow[4];
        _robot->sensors.yellow_imu->get_quat(qYellow);
        debug() << "qyellow: " << qYellow[0] << "; " << qYellow[1] << "; " << qYellow[2] << "; " << qYellow[3];

        if (qYellow[0] < 5E-5) {
            std::cout << "Yellow IMU was not correctly tared." << std::endl;
            critical() << "Yellow IMU was not correctly tared.";
            _robot->user_feedback.buzzer->makeNoise(Buzzer::ERROR_BUZZ);
        } else {
            _robot->user_feedback.buzzer->makeNoise(Buzzer::TRIPLE_BUZZ);
        }
    }
}

void myo_2electrodes::tare_redIMU()
{
    if (!_robot->sensors.red_imu) {
        std::cout << "Red IMU is missing." << std::endl;
        critical() << "Red IMU is missing.";
        _robot->user_feedback.buzzer->makeNoise(Buzzer::ERROR_BUZZ);
    } else {
        _robot->sensors.red_imu->send_command_algorithm_init_then_tare();

        debug("Wait ...");

        usleep(6 * 1000000);

        double qRed[4];
        _robot->sensors.red_imu->get_quat(qRed);
        debug() << "qred: " << qRed[0] << "; " << qRed[1] << "; " << qRed[2] << "; " << qRed[3];

        if (qRed[0] < 5E-5) {
            std::cout << "Red IMU was not correctly tared." << std::endl;
            critical() << "Red IMU was not correctly tared.";
            _robot->user_feedback.buzzer->makeNoise(Buzzer::ERROR_BUZZ);
        } else {
            _robot->user_feedback.buzzer->makeNoise(Buzzer::TRIPLE_BUZZ);
        }
    }
}

void myo_2electrodes::calibrations()
{
    // ELBOW
    if (_robot->joints.elbow_flexion->is_calibrated() == false) {
        _robot->joints.elbow_flexion->calibrate();
    }
    if (_robot->joints.elbow_flexion->is_calibrated())
        debug() << "Calibration elbow: ok \n";

//    // WRIST PRONATION
//    if (_robot->joints.wrist_pronation->is_calibrated() == false) {
//        _robot->joints.wrist_pronation->calibrate();
//    }
//    if (_robot->joints.wrist_pronation->is_calibrated())
//        debug() << "Calibration wrist pronation: ok \n";

    if (protoCyb)
        _robot->joints.elbow_flexion->move_to(100, 20, true);
    else
        _robot->joints.elbow_flexion->move_to(-90, 20, true);

}

void myo_2electrodes::elbowTo90()
{
    // ELBOW
    if (_robot->joints.elbow_flexion->is_calibrated() == false) {
        warning() << "Elbow not calibrated...";
        return;
    } else {
        if (protoCyb)
            _robot->joints.elbow_flexion->move_to(100, 20);
        else
            _robot->joints.elbow_flexion->move_to(-90,20);
    }
}

void myo_2electrodes::readAllADC() //Optimized function to read all 6 electrodes
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
    //    uint16_t config_c1 = config_global | ADS1015_REG_CONFIG_MUX_SINGLE_1; //for channel 1
    uint16_t config_c2 = config_global | ADS1015_REG_CONFIG_MUX_SINGLE_2; //for channel 2
    uint16_t config_c3 = config_global | ADS1015_REG_CONFIG_MUX_SINGLE_3; //for channel 3

    //Write config register to the ADC
    _robot->sensors.adc0->writeRegister(ADS1015_REG_POINTER_CONFIG, config_c2);
    //    _robot->sensors.adc2->writeRegister(ADS1015_REG_POINTER_CONFIG, config_c2);
    _robot->sensors.adc3->writeRegister(ADS1015_REG_POINTER_CONFIG, config_c2);

    // Wait for the conversion to complete
    while (!(_robot->sensors.adc0->readRegister(ADS1015_REG_POINTER_CONFIG) >> 15))
        ;

    // Read the conversion results
    _electrodes[0] = _robot->sensors.adc0->readRegister(ADS1015_REG_POINTER_CONVERT);
    //    _electrodes[2] = _robot->sensors.adc2->readRegister(ADS1015_REG_POINTER_CONVERT);
    _electrodes[4] = _robot->sensors.adc3->readRegister(ADS1015_REG_POINTER_CONVERT);

    //Write config register to the ADC
    _robot->sensors.adc0->writeRegister(ADS1015_REG_POINTER_CONFIG, config_c3);
    //    _robot->sensors.adc2->writeRegister(ADS1015_REG_POINTER_CONFIG, config_c0);
    _robot->sensors.adc3->writeRegister(ADS1015_REG_POINTER_CONFIG, config_c0);

    // Wait for the conversion to complete
    while (!(_robot->sensors.adc0->readRegister(ADS1015_REG_POINTER_CONFIG) >> 15))
        ;

    // Read the conversion results
    _electrodes[1] = _robot->sensors.adc0->readRegister(ADS1015_REG_POINTER_CONVERT);
    //    _electrodes[3] = _robot->sensors.adc2->readRegister(ADS1015_REG_POINTER_CONVERT);
    _electrodes[5] = _robot->sensors.adc3->readRegister(ADS1015_REG_POINTER_CONVERT);

    for (uint16_t i = 0; i < _n_electrodes; i++) {
        if (_electrodes[i] > 60000) {
            _electrodes[i] = 0;
        }
    }
}

bool myo_2electrodes::setup()
{
    _robot->joints.wrist_pronation->set_encoder_position(0);

    if (saveData) {
        // OPEN AND NAME DATA FILE
        std::string filename("/opt/myo2");
        std::string suffix;
        int cnt = 0;
        std::string extension(".txt");
        do {
            ++cnt;
            suffix = "_" + std::to_string(cnt);
        } while (std::filesystem::exists(filename + suffix + extension));
        _file = std::ofstream(filename + suffix + extension);
        if (!_file.good()) {
            critical() << "Failed to open " << (filename + suffix + extension);
            return false;
        }

        _need_to_write_header = true;
    }
    _cnt = 0;

    _pin1 = 10;
    _pin2 = 10;
    _start_time = clock::now();
    return true;
}

void myo_2electrodes::loop(double, clock::time_point time)
{
    static double pronoSupEncoder = 0;
    static double elbowEncoder = 0;
    static double tmpEncoder = 0;

    if (_robot->joints.elbow_flexion->is_calibrated() == false) {

        static bool first = true;
        if (first) {
            printf("Elbow is not calibrated. Enter 'stop' \n");
            first = false;
        }
    } else {
#if OPTITRACK
        _robot->sensors.optitrack->update();
        optitrack_data_t data = _robot->sensors.optitrack->get_last_data();
        if (_cnt == 0) {
            debug() << "Rigid Bodies: " << data.nRigidBodies;
            _cnt = 1;
        }
#endif

        double timeWithDelta = (time - _start_time).count();

        static std::unique_ptr<MyoControl::Classifier> myocontrol;

        static const unsigned int counts_after_mode_change = 15;
        static const unsigned int counts_cocontraction = 5;
        static const unsigned int counts_before_bubble = 2;
        static const unsigned int counts_after_bubble = 10;

        static const MyoControl::EMGThresholds thresholds(_forward_upper, _forward_lower, _forward_upper, _backward_upper, _backward_lower, _backward_upper);

        auto robot = _robot;

        static LedStrip::color current_color = LedStrip::none;

        if (protoCyb){
            MyoControl::Action elbow(
                        "Elbow", [robot]() { robot->joints.elbow_flexion->set_velocity_safe(-25); }, [robot]() { robot->joints.elbow_flexion->set_velocity_safe(25); }, [robot]() { robot->joints.elbow_flexion->set_velocity_safe(0); });
            MyoControl::Action wrist_pronosup(
                        "Wrist rotation", [robot]() { robot->joints.wrist_pronation->set_velocity_safe(-80); }, [robot]() { robot->joints.wrist_pronation->set_velocity_safe(80); }, [robot]() { robot->joints.wrist_pronation->set_velocity_safe(0); });

            std::vector<MyoControl::Action> s1{ wrist_pronosup, elbow };

            static bool first = true;
            if (first) {
                current_color = LedStrip::green;
                myocontrol = std::make_unique<MyoControl::BubbleCocoClassifier>(s1, thresholds, counts_after_mode_change, counts_cocontraction, counts_before_bubble, counts_after_bubble);
                first = false;
            }

            readAllADC();
            /// MYO CONTROL FOR WRIST AND ELBOW
            myocontrol->process(_electrodes[0], _electrodes[1]);
        }
        else {
            MyoControl::Action elbow(
                        "Elbow", [robot]() { robot->joints.elbow_flexion->set_velocity_safe(25); }, [robot]() { robot->joints.elbow_flexion->set_velocity_safe(-25); }, [robot]() { robot->joints.elbow_flexion->set_velocity_safe(0); });
            MyoControl::Action wrist_pronosup(
                        "Wrist rotation", [robot]() { robot->joints.wrist_pronation->set_velocity_safe(-80); }, [robot]() { robot->joints.wrist_pronation->set_velocity_safe(80); }, [robot]() { robot->joints.wrist_pronation->set_velocity_safe(0); });

            std::vector<MyoControl::Action> s1{ wrist_pronosup, elbow };

            static bool first = true;
            if (first) {
                current_color = LedStrip::green;
                myocontrol = std::make_unique<MyoControl::BubbleCocoClassifier>(s1, thresholds, counts_after_mode_change, counts_cocontraction, counts_before_bubble, counts_after_bubble);
                first = false;
            }

            readAllADC();
            /// MYO CONTROL FOR WRIST AND ELBOW
            myocontrol->process(_electrodes[4], _electrodes[5]);
        }

        if (_cnt % 50 == 0)
            debug() << "electrodes 0 et 1: " << _electrodes[4] << " " << _electrodes[5];

        ///PUSH-BUTTONS FOR HAND CONTROL
        int pin_down_value = 0, pin_up_value = 0;
        //        debug() << "electrodes 4 et 5: " << _electrodes[4] << " " << _electrodes[5];
        std::cout << _electrodes[0] << "\t" << _electrodes[1] << "\t" << _electrodes[4] << "\t" << _electrodes[5] << std::endl;
        if (protoCyb){
            if (_electrodes[5] <= 0 && _electrodes[4] > 0) {
                // Open quantum hand
                _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 2, 2);
            } else if (_electrodes[4] <= 0 && _electrodes[5] > 0) {
                // Close quantum hand
                _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 1, 2);
            } else {
                _robot->joints.hand_quantum->makeContraction(QuantumHand::STOP);
            }
        }
        else{
            pin_down_value = _robot->btn2;
            pin_up_value = _robot->btn1;
            static int prev_pin_up_value = 1, prev_pin_down_value = 1;
            if (!_robot->joints.hand) {
                // printf("Quantum hand \n");
                std::cout << pin_down_value << "\t" << pin_up_value << std::endl;
                if (pin_down_value == 0 && pin_up_value == 1) {
                    // close hand
                    _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 2, 2);
                } else if (pin_up_value == 0 && pin_down_value == 1) {
                    //open hand
                    _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 1, 2);
                } else {
                    _robot->joints.hand_quantum->makeContraction(QuantumHand::STOP);
                }
            } else {
                printf("TB hand\n");
                if (pin_down_value == 0 && prev_pin_down_value == 1) {
                    _robot->joints.hand->move(TouchBionicsHand::PINCH_CLOSING);
                } else if (pin_up_value == 0 && prev_pin_up_value == 1) {
                    _robot->joints.hand->move(TouchBionicsHand::PINCH_OPENING);
                } else if ((pin_down_value == 1 && pin_up_value == 1) && (prev_pin_down_value == 0 || prev_pin_up_value == 0)) {
                    _robot->joints.hand->move(TouchBionicsHand::STOP);
                }
            }

            prev_pin_down_value = pin_down_value;
            prev_pin_up_value = pin_up_value;
        }
        /// ELBOW
        try {
            tmpEncoder = elbowEncoder;
            elbowEncoder = _robot->joints.elbow_flexion->read_encoder_position();
        } catch (std::runtime_error& e) {
            std::cout << "Runtime error when reading elbow encoder: " << e.what() << std::endl;
            elbowEncoder = tmpEncoder;
        }
        /// WRIST
        try {
            tmpEncoder = pronoSupEncoder;
            pronoSupEncoder = _robot->joints.wrist_pronation->read_encoder_position();
        } catch (std::runtime_error& e) {
            std::cout << "Runtime error when reading wrist encoder: " << e.what() << std::endl;
            pronoSupEncoder = tmpEncoder;
        }
        //    debug() << "pronosup encoder: " << pronoSupEncoder;

        double qWhite[4], qRed[4], qYellow[4];
        if (_robot->sensors.white_imu) {
            _robot->sensors.white_imu->get_quat(qWhite);
        }
        if (_robot->sensors.red_imu) {
            _robot->sensors.red_imu->get_quat(qRed);
            //        debug() << "qRed: " << _qHip.w() << "; " << _qHip.x() << "; " << _qHip.y() << "; " << _qHip.z();
        }
        if (_robot->sensors.yellow_imu) {
            _robot->sensors.yellow_imu->get_quat(qYellow);
            //        debug() << "qyellow: " << qYellow[0] << "; " << qYellow[1] << "; " << qYellow[2] << "; " << qYellow[3];
        }

        double angles[2];
        if (protoCyb){
            angles[1] = -elbowEncoder / _robot->joints.elbow_flexion->r_incs_per_deg(); // for Cybathlon prototype
            angles[0] = -pronoSupEncoder / _robot->joints.wrist_pronation->r_incs_per_deg();
        }
        else{
            angles[1] = elbowEncoder / _robot->joints.elbow_flexion->r_incs_per_deg(); // for Cybathlon prototype
            angles[0] = -pronoSupEncoder / _robot->joints.wrist_pronation->r_incs_per_deg();
        }

        if (myocontrol->has_changed_mode()) {
            _robot->user_feedback.buzzer->makeNoise(Buzzer::STANDARD_BUZZ);
        }

        std::vector<LedStrip::color> colors(10, current_color);
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

        int btnStart = 0;
        if (saveData) {
            /// WRITE DATA
            _file << timeWithDelta << ' ' << btnStart << ' ' << _electrodes[0] << ' ' << _electrodes[1] << ' ' << _electrodes[4] << ' ' << _electrodes[5];
            _file << ' ' << qRed[0] << ' ' << qRed[1] << ' ' << qRed[2] << ' ' << qRed[3] << ' ' << qWhite[0] << ' ' << qWhite[1] << ' ' << qWhite[2] << ' ' << qWhite[3];
            _file << ' ' << qYellow[0] << ' ' << qYellow[1] << ' ' << qYellow[2] << ' ' << qYellow[3];
            _file << ' ' << pronoSupEncoder << ' ' << elbowEncoder << ' ' << angles[0] << ' ' << angles[1];

#if OPTITRACK
            _file << ' ' << data.nRigidBodies;
            for (unsigned int i = 0; i < data.nRigidBodies; i++) {
                _file << ' ' << data.rigidBodies[i].ID << ' ' << data.rigidBodies[i].bTrackingValid << ' ' << data.rigidBodies[i].fError;
                _file << ' ' << data.rigidBodies[i].qw << ' ' << data.rigidBodies[i].qx << ' ' << data.rigidBodies[i].qy << ' ' << data.rigidBodies[i].qz;
                _file << ' ' << data.rigidBodies[i].x << ' ' << data.rigidBodies[i].y << ' ' << data.rigidBodies[i].z;
            }
#endif

            _file << std::endl;
        }
    }
    //Publish EMG values with MQTT
    for (uint16_t i = 0; i < _n_electrodes; i++) {
        _mqtt.publish("sam/emg/time/" + std::to_string(i), std::to_string(_electrodes[i]));
    }

    _cnt++;
}

void myo_2electrodes::cleanup()
{
    _robot->joints.wrist_pronation->forward(0);

    //_robot->joints.elbow_flexion->move_to(0, 20);
    //    _robot->joints.elbow_flexion->move_to(0, 10);

    _robot->user_feedback.leds->set(LedStrip::white, 10);
}
