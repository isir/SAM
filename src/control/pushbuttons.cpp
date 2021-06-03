#include "pushbuttons.h"
#include "algo/myocontrol.h"
#include "ui/visual/ledstrip.h"
#include "utils/check_ptr.h"
#include "utils/log/log.h"
#include <filesystem>

// indicate if optitrack is on
#define OPTITRACK 1

pushButtons::pushButtons(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("PushButtons", .01)
    , _robot(robot)
{
    if (!check_ptr(_robot->joints.elbow_flexion, _robot->joints.wrist_pronation, _robot->joints.hand)) {
        throw std::runtime_error("PushButtons is missing components");
    }

    _menu->set_description("PushButtons");
    _menu->set_code("pb");

    _menu->add_item(_robot->joints.elbow_flexion->menu());
    _menu->add_item(_robot->joints.wrist_pronation->menu());
    _menu->add_item(_robot->joints.hand->menu());
    _menu->add_item("tare", "Tare IMUs", [this](std::string) { this->tare_IMU(); });
}

pushButtons::~pushButtons()
{
    stop_and_join();
}

void pushButtons::tare_IMU()
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

bool pushButtons::setup()
{
    //_robot->joints.hand->take_ownership();
    //_robot->joints.hand->init_sequence();
    _robot->joints.elbow_flexion->calibrate();
    _robot->joints.wrist_pronation->calibrate();

    if (saveData) {
        // OPEN AND NAME DATA FILE
        std::string filename("/opt/pushButtons");
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
    _start_time = clock::now();
    return true;
}

void pushButtons::loop(double, clock::time_point time)
{

#if OPTITRACK
    _robot->sensors.optitrack->update();
    optitrack_data_t data = _robot->sensors.optitrack->get_last_data();
    if (_cnt == 0) {
        debug() << "Rigid Bodies: " << data.nRigidBodies;
        _cnt = 1;
    }
#endif

    double timeWithDelta = (time - _start_time).count();

    double debugData[40];

    static std::unique_ptr<MyoControl::Classifier> myocontrol;

    static int control_mode = 0;
    static int counter_auto_control = 0, move_elbow_counter = 0;
    static const int max_acc_change_mode = 15;

    static const unsigned int counts_after_mode_change = 15;
    static const unsigned int counts_cocontraction = 5;
    static const unsigned int counts_before_bubble = 5;
    static const unsigned int counts_after_bubble = 5;

    static const MyoControl::EMGThresholds thresholds(15, 8, 15, 15, 8, 15);

    auto robot = _robot;
    MyoControl::Action elbow(
        "Elbow", [robot]() { robot->joints.elbow_flexion->set_velocity_safe(-35); }, [robot]() { robot->joints.elbow_flexion->set_velocity_safe(35); }, [robot]() { robot->joints.elbow_flexion->set_velocity_safe(0); });
    MyoControl::Action wrist_pronosup(
        "Wrist rotation", [robot]() { robot->joints.wrist_pronation->set_velocity_safe(40); }, [robot]() { robot->joints.wrist_pronation->set_velocity_safe(-40); }, [robot]() { robot->joints.wrist_pronation->set_velocity_safe(0); });
    MyoControl::Action hand(
        "Hand", [robot]() { robot->joints.hand->move(TouchBionicsHand::HAND_OPENING_ALL); }, [robot]() { robot->joints.hand->move(TouchBionicsHand::HAND_CLOSING_ALL); }, [robot]() { robot->joints.hand->move(TouchBionicsHand::STOP); });

    std::vector<MyoControl::Action> s1{ wrist_pronosup, elbow };

    std::vector<MyoControl::Action> s2{ hand, wrist_pronosup };

    static LedStrip::color current_color = LedStrip::none;

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
        _file << ' ' << pronoSupEncoder << ' ' << elbowEncoder << ' ' << emg[0] << ' ' << emg[1];

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

void pushButtons::cleanup()
{
    _robot->joints.wrist_pronation->forward(0);

    //_robot->joints.elbow_flexion->move_to(0, 20);
    _robot->joints.elbow_flexion->move_to(0, 10);

    _robot->user_feedback.leds->set(LedStrip::white, 10);
    _robot->joints.hand->release_ownership();
}
