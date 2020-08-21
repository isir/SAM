#include "myo_2electrodes.h"
#include "algo/myocontrol.h"
#include "ui/visual/ledstrip.h"
#include "utils/check_ptr.h"
#include "utils/log/log.h"
#include <filesystem>

#include "wiringPi.h"

// indicate if optitrack is on
#define OPTITRACK 1

myo_2electrodes::myo_2electrodes(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("myo_2electrodes", .01)
    , _robot(robot)
{
    if (!check_ptr(_robot->joints.elbow_flexion, _robot->joints.wrist_pronation)) {
        throw std::runtime_error("myo_2electrodes is missing components");
    }

    _menu->set_description("myo_2electrodes");
    _menu->set_code("myo2");

    _menu->add_item(_robot->joints.elbow_flexion->menu());
    _menu->add_item(_robot->joints.wrist_pronation->menu());
    _menu->add_item("tareAll", "Tare all IMUs", [this](std::string) { this->tare_allIMU(); });
    _menu->add_item("tareWhite", "Tare white IMUs", [this](std::string) { this->tare_whiteIMU(); });
    _menu->add_item("tareYellow", "Tare yellow IMUs", [this](std::string) { this->tare_yellowIMU(); });
    _menu->add_item("calib", "Calibrations", [this](std::string) { this->calibrations(); });
}

myo_2electrodes::~myo_2electrodes()
{
    stop_and_join();
}

void myo_2electrodes::tare_allIMU()
{
    if (_robot->sensors.white_imu)
        _robot->sensors.white_imu->send_command_algorithm_init_then_tare();
    if (_robot->sensors.red_imu)
        _robot->sensors.red_imu->send_command_algorithm_init_then_tare();
    if (_robot->sensors.yellow_imu)
        _robot->sensors.yellow_imu->send_command_algorithm_init_then_tare();

    debug("Wait ...");

    usleep(6 * 1000000);
    _robot->user_feedback.buzzer->makeNoise(Buzzer::TRIPLE_BUZZ);
}

void myo_2electrodes::tare_whiteIMU()
{
    if (_robot->sensors.white_imu)
        _robot->sensors.white_imu->send_command_algorithm_init_then_tare();

    debug("Wait ...");

    usleep(6 * 1000000);
    _robot->user_feedback.buzzer->makeNoise(Buzzer::TRIPLE_BUZZ);
}

void myo_2electrodes::tare_yellowIMU()
{
    if (_robot->sensors.yellow_imu)
        _robot->sensors.yellow_imu->send_command_algorithm_init_then_tare();

    debug("Wait ...");

    usleep(6 * 1000000);
    _robot->user_feedback.buzzer->makeNoise(Buzzer::TRIPLE_BUZZ);
}

void myo_2electrodes::calibrations()
{
    // ELBOW
    if (_robot->joints.elbow_flexion->is_calibrated() == false) {
        _robot->joints.elbow_flexion->calibrate();
    }
    if (_robot->joints.elbow_flexion->is_calibrated())
        debug() << "Calibration elbow: ok \n";

    // WRIST PRONATION
    if (_robot->joints.wrist_pronation->is_calibrated() == false) {
        _robot->joints.wrist_pronation->calibrate();
    }
    if (_robot->joints.wrist_pronation->is_calibrated())
        debug() << "Calibration wrist pronation: ok \n";
}

bool myo_2electrodes::setup()
{
    _robot->joints.wrist_pronation->set_encoder_position(0);

    // ADC setup
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
}

void myo_2electrodes::loop(double, clock::time_point time)
{

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

        // button "mode compensation" of cybathlon to indicate beginning of motion
        int btnStart;
        if (!_robot->btn3)
            btnStart = 0;
        else
            btnStart = 1;

        double timeWithDelta = (time - _start_time).count();

        static std::unique_ptr<MyoControl::Classifier> myocontrol;
        static std::unique_ptr<MyoControl::Classifier> handcontrol;

        static const unsigned int counts_after_mode_change = 15;
        static const unsigned int counts_btn = 2;
        static const unsigned int counts_cocontraction = 5;
        static const unsigned int counts_before_bubble = 2;
        static const unsigned int counts_after_bubble = 10;

        static const MyoControl::EMGThresholds thresholds(3500, 2200, 2200, 3500, 2200, 2200);

        auto robot = _robot;
        MyoControl::Action elbow(
            "Elbow", [robot]() { robot->joints.elbow_flexion->set_velocity_safe(-25); }, [robot]() { robot->joints.elbow_flexion->set_velocity_safe(25); }, [robot]() { robot->joints.elbow_flexion->set_velocity_safe(0); });
        MyoControl::Action wrist_pronosup(
            "Wrist rotation", [robot]() { robot->joints.wrist_pronation->set_velocity_safe(80); }, [robot]() { robot->joints.wrist_pronation->set_velocity_safe(-80); }, [robot]() { robot->joints.wrist_pronation->set_velocity_safe(0); });

        std::vector<MyoControl::Action> s1{ wrist_pronosup, elbow };

        static LedStrip::color current_color = LedStrip::none;

        static bool first = true;
        if (first) {
            current_color = LedStrip::green;
            myocontrol = std::make_unique<MyoControl::BubbleCocoClassifier>(s1, thresholds, counts_after_mode_change, counts_cocontraction, counts_before_bubble, counts_after_bubble);
            first = false;
        }

        readAllADC();
        myocontrol->process(_electrodes[0], _electrodes[1]);
        //        debug() << "electrodes 4 et 5: " << _electrodes[4] << " " << _electrodes[5];
        if (_cnt % 50 == 0)
            debug() << "electrodes 0 et 1: " << _electrodes[0] << " " << _electrodes[1];

        if (_electrodes[4] == 0 && _electrodes[5] != 0) {
            // Open quantum hand
            _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 1, 2);
        } else if (_electrodes[5] == 0 && _electrodes[4] != 0) {
            // Close quantum hand
            _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 2, 2);
        } else {
            // rien
        }

        /// ELBOW
        double elbowEncoder = _robot->joints.elbow_flexion->read_encoder_position();
        /// WRIST
        double pronoSupEncoder = _robot->joints.wrist_pronation->read_encoder_position();
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

        if (saveData) {
            /// WRITE DATA
            _file << timeWithDelta << ' ' << btnStart;
            _file << ' ' << qWhite[0] << ' ' << qWhite[1] << ' ' << qWhite[2] << ' ' << qWhite[3] << ' ' << qRed[0] << ' ' << qRed[1] << ' ' << qRed[2] << ' ' << qRed[3];
            _file << ' ' << qYellow[0] << ' ' << qYellow[1] << ' ' << qYellow[2] << ' ' << qYellow[3];
            _file << ' ' << pronoSupEncoder << ' ' << elbowEncoder << ' ' << _electrodes[0] << ' ' << _electrodes[1] << ' ' << _electrodes[4] << ' ' << _electrodes[5];

#if OPTITRACK
            _file << ' ' << data.nRigidBodies;
            for (int i = 0; i < data.nRigidBodies; i++) {
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
    _robot->joints.elbow_flexion->move_to(0, 10);

    _robot->user_feedback.leds->set(LedStrip::white, 10);
}
