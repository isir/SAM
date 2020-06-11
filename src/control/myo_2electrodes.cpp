#include "myo_2electrodes.h"
#include "algo/myocontrol.h"
#include "ui/visual/ledstrip.h"
#include "utils/check_ptr.h"
#include "utils/log/log.h"
#include <filesystem>

#include "wiringPi.h"

// indicate if optitrack is on
#define OPTITRACK 0

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
    _menu->add_item("tare", "Tare IMUs", [this](std::string) { this->tare_IMU(); });
    _menu->add_item("calib", "Calibrations", [this](std::string) { this->calibrations(); });
}

myo_2electrodes::~myo_2electrodes()
{
    stop_and_join();
}

void myo_2electrodes::tare_IMU()
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
        std::string filename("myo2");
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
    _start_time = clock::now();
    return true;
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
        if (_cnt == 0)
            debug() << "Rigid Bodies: " << data.nRigidBodies;
#endif

        double timeWithDelta = (time - _start_time).count();

        static std::unique_ptr<MyoControl::Classifier> myocontrol;

        static const unsigned int counts_after_mode_change = 15;
        static const unsigned int counts_btn = 2;
        static const unsigned int counts_cocontraction = 5;
        static const unsigned int counts_before_bubble = 2;
        static const unsigned int counts_after_bubble = 10;

        static const MyoControl::EMGThresholds thresholds(5000, 2200, 0, 5000, 2200, 0);

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

        _emg[0] = _robot->sensors.adc0->readADC_SingleEnded(2);
        _emg[1] = _robot->sensors.adc0->readADC_SingleEnded(3);

        myocontrol->process(_emg[0], _emg[1]);

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
            _file << timeWithDelta;
            _file << ' ' << qWhite[0] << ' ' << qWhite[1] << ' ' << qWhite[2] << ' ' << qWhite[3] << ' ' << qRed[0] << ' ' << qRed[1] << ' ' << qRed[2] << ' ' << qRed[3];
            _file << ' ' << qYellow[0] << ' ' << qYellow[1] << ' ' << qYellow[2] << ' ' << qYellow[3];
            _file << ' ' << pronoSupEncoder << ' ' << elbowEncoder << ' ' << _emg[0] << ' ' << _emg[1];

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
}

void myo_2electrodes::cleanup()
{
    _robot->joints.wrist_pronation->forward(0);

    //_robot->joints.elbow_flexion->move_to(0, 20);
    _robot->joints.elbow_flexion->move_to(0, 10);

    _robot->user_feedback.leds->set(LedStrip::white, 10);
}
