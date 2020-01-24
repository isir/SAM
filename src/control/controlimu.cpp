#include "controlimu.h"
#include "algo/myocontrol.h"
#include "utils/check_ptr.h"
#include "utils/log/log.h"
#include "wiringPi.h"
#include <filesystem>
#include <iostream>

// indicate if optitrack is on
#define OPTITRACK 0

ControlIMU::ControlIMU(std::string name, std::string filename, std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop(name, 0.01)
    , _robot(robot)
    , _k("k", BaseParam::ReadWrite, this, 5)
    , _lt("lt(cm)", BaseParam::ReadWrite, this, 40)
    , _lsh("lsh(cm)", BaseParam::ReadWrite, this, 20)
    , _pin_up(24)
    , _pin_down(22)
    , _lua("lua(cm)", BaseParam::ReadWrite, this, 30)
    , _lfa("lfa(cm)", BaseParam::ReadWrite, this, 20)
    , _lwrist("lwrist(cm)", BaseParam::ReadWrite, this, 10)
    , _lambdaE("lambda elbow", BaseParam::ReadWrite, this, 0)
    , _lambdaWF("lambda wrist flex", BaseParam::ReadWrite, this, 0)
    , _lambdaWPS("lambda wrist PS", BaseParam::ReadWrite, this, 0)
    , _thresholdE("threshold E", BaseParam::ReadWrite, this, 5)
    , _thresholdWF("threshold WF", BaseParam::ReadWrite, this, 5)
    , _thresholdWPS("threshold WPS", BaseParam::ReadWrite, this, 5)
{
    if (!check_ptr(_robot->joints.elbow_flexion, _robot->joints.wrist_pronation, _robot->sensors.white_imu, _robot->sensors.yellow_imu)) {
        throw std::runtime_error("Jacobian Formulation Control is missing components");
    }

    _menu->add_item("tare", "Tare IMUs", [this](std::string) { this->tare_IMU(); });
    _menu->add_item("analog", "Show IMU Analog data", [this](std::string) { this->analog_IMU(); });
    _menu->add_item("calib", "Calibration", [this](std::string) { this->calibrations(); });

    _menu->add_item(_robot->joints.elbow_flexion->menu());
    _menu->add_item(_robot->joints.wrist_pronation->menu());
    if (_robot->joints.hand)
        _menu->add_item(_robot->joints.hand->menu());

    if (_robot->joints.wrist_flexion) {
        _menu->add_item(_robot->joints.wrist_flexion->menu());

        _nbDOF = 3; // set the number of dof of the prosthetic arm to adapt intermediate computations in control law
        debug() << "nDOF: " << _nbDOF;
    } else {
        _nbDOF = 2;
        debug() << "nDOF: " << _nbDOF;
    }

    pullUpDnControl(_pin_up, PUD_UP);
    pullUpDnControl(_pin_down, PUD_UP);

    _filename = filename;

    // threshold EMG
    _th_low[0] = 3000;
    _th_low[1] = 1500;

    _th_high[0] = 3000;
    _th_high[1] = 3000;
}

ControlIMU::~ControlIMU()
{
    _robot->joints.elbow_flexion->forward(0);
    _robot->joints.wrist_pronation->forward(0);
    stop_and_join();
}

void ControlIMU::analog_IMU()
{
    double a[8];
    _robot->sensors.white_imu->get_analog(a);
}

void ControlIMU::tare_IMU()
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

void ControlIMU::calibrations()
{
    // HAND
    //    if (_robot->joints.hand) {
    //        _robot->joints.hand->take_ownership();
    //        _robot->joints.hand->init_sequence();
    //        //        _robot->joints.hand->move(14);
    //    }
    // ELBOW
    if (_robot->joints.elbow_flexion->is_calibrated() == false) {
        _robot->joints.elbow_flexion->calibrate();
    }
    if (_robot->joints.elbow_flexion->is_calibrated())
        debug() << "Calibration elbow: ok \n";

    // WRIST FLEXION
    if (_robot->joints.wrist_flexion) {
        if (_robot->joints.wrist_flexion->is_calibrated() == false) {
            _robot->joints.wrist_flexion->calibrate();
        }
        if (_robot->joints.wrist_flexion->is_calibrated())
            debug() << "Calibration wrist flexion: ok \n";
    }

    // WRIST PRONATION
    if (_robot->joints.wrist_pronation->is_calibrated() == false) {
        _robot->joints.wrist_pronation->calibrate();
    }
    if (_robot->joints.wrist_pronation->is_calibrated())
        debug() << "Calibration wrist pronation: ok \n";

    if (protoCyb)
        _robot->joints.elbow_flexion->move_to(90, 20);
    else
        _robot->joints.elbow_flexion->move_to(-90, 20);
}

void ControlIMU::displayPin()
{
    int pin_down_value = digitalRead(_pin_down);
    int pin_up_value = digitalRead(_pin_up);
    debug() << "PinUp: " << pin_up_value;
    debug() << "PinDown: " << pin_down_value;
}

bool ControlIMU::setup()
{
    // Check for calibration
    //    if (_robot->joints.wrist_flexion) {
    //        if ((_robot->joints.elbow_flexion->is_calibrated() == false) || (_robot->joints.wrist_flexion->is_calibrated() == false) || (_robot->joints.wrist_pronation->is_calibrated() == false)) {
    //            debug() << "Actuators not calibrated";
    //            return false;
    //        }
    //    } else {
    //        if ((_robot->joints.elbow_flexion->is_calibrated() == false)) {
    //            debug() << "Elbow not calibrated";
    //            return false;
    //        }
    //    }

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
        std::string suffix;

        int cnt = 0;
        std::string extension(".txt");
        do {
            ++cnt;
            suffix = "_" + std::to_string(cnt);
        } while (std::filesystem::exists(_filename + suffix + extension));

        _file = std::ofstream(_filename + suffix + extension);
        if (!_file.good()) {
            critical() << "Failed to open" << (_filename + suffix + extension);
            return false;
        }
        _need_to_write_header = true;
    }
    _start_time = clock::now();

    // INITIALIZATION
    _cnt = 0;
    for (int i = 0; i < nbLinks; i++) {
        _theta[i] = 0.;
    }
    return true;
}

void ControlIMU::loop(double, clock::time_point time)
{

#if OPTITRACK
    _robot->sensors.optitrack->update();
    optitrack_data_t data = _robot->sensors.optitrack->get_last_data();
    debug() << "Rigid Bodies: " << data.nRigidBodies;
#endif

    double timeWithDelta = (time - _start_time).count();

    double debugData[40];

    if (saveData) {
        /// WRITE FILE HEADERS
        if (_need_to_write_header) {
            _file << " time, pinUp, pinDown,";
            _file << " qWhite.w, qWhite.x, qWhite.y, qWhite.z, qTronc.w, qTronc.x, qTronc.y, qTronc.z,";
            _file << " qYellow.w, qYellow.x, qYellow.y, qYellow.z,";
            _file << " to complete,";
            _file << "\r\n";
            _need_to_write_header = false;
        }
    }

    /// HAND
    if (protoCyb) {
        _emg[0] = _robot->sensors.adc0->readADC_SingleEnded(2);
        _emg[1] = _robot->sensors.adc0->readADC_SingleEnded(3);
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

        //EMG1 and EMG2 = hand
        static bool btn = 0;
        if (!_robot->btn1) {
            btn = 1;
        } else {
            btn = 0;
        }
        handcontrol->process(_emg[0], _emg[1], btn);
    }

    /// ELBOW
    double elbowEncoder = _robot->joints.elbow_flexion->read_encoder_position();
    /// WRIST
    double pronoSupEncoder = _robot->joints.wrist_pronation->read_encoder_position();
    //    debug() << "pronosup encoder: " << pronoSupEncoder;
    double wristFlexEncoder = 0.;

    /// PROTO with wrist flexor
    if (_robot->joints.wrist_flexion) {
        // set thresholds
        _threshold[0] = _thresholdWF * M_PI / 180;
        _threshold[1] = _thresholdWPS * M_PI / 180;
        _threshold[2] = _thresholdE * M_PI / 180;
        // set gain
        _lambda[0] = _lambdaWF;
        _lambda[1] = _lambdaWPS;
        _lambda[2] = _lambdaE;
        // set arm lengths
        _l[0] = _lwrist; //  from the pronosupination joint to the wrist flexion/ext joint
        _l[1] = _lfa; //  from the wrist flex/ext joint to the elbow flex/ext joint
        _l[2] = _lua; // from the elbow joint to the acromion/shoulder joint

        wristFlexEncoder = _robot->joints.wrist_flexion->read_encoder_position();
        _theta[0] = -wristFlexEncoder / _robot->joints.wrist_flexion->r_incs_per_deg();
        _theta[1] = -pronoSupEncoder / _robot->joints.wrist_pronation->r_incs_per_deg();
        _theta[2] = -elbowEncoder / _robot->joints.elbow_flexion->r_incs_per_deg();
        if (_cnt % 50 == 0)
            debug() << "_theta(deg): " << _theta[0] << ", " << _theta[1] << ", " << _theta[2] << "\r\n";
        _theta[0] = -M_PI / 2 + _theta[0] * M_PI / 180;
        _theta[1] = M_PI / 2 + _theta[1] * M_PI / 180;
        _theta[2] = M_PI / 2 + _theta[2] * M_PI / 180;

    }
    /// PROTO without wrist flexor
    else {
        // set thresholds
        _threshold[0] = _thresholdWPS * M_PI / 180;
        _threshold[1] = _thresholdE * M_PI / 180;

        // set gain
        _lambda[0] = _lambdaWPS;
        _lambda[1] = _lambdaE;

        // set arm lengths
        _l[0] = _lwrist;
        _l[1] = _lfa;
        _l[2] = _lua;

        _theta[0] = -pronoSupEncoder / _robot->joints.wrist_pronation->r_incs_per_deg();
        if (protoCyb)
            _theta[1] = -elbowEncoder / _robot->joints.elbow_flexion->r_incs_per_deg();
        else
            _theta[1] = elbowEncoder / _robot->joints.elbow_flexion->r_incs_per_deg();
        if (_cnt % 50 == 0)
            debug() << "_theta(deg): " << _theta[0] << ", " << _theta[1] << "\r\n";
        // in radians:
        _theta[0] = _theta[0] * M_PI / 180;
        _theta[1] = _theta[1] * M_PI / 180;

        _lambda[0] = _lambdaWPS;
        _lambda[1] = _lambdaE;
    }

    // create and set posA and posHip to zero. posHip is useless but need to be defined for some _lawJ functions
    Eigen::Vector3d posA = Eigen::Vector3d::Zero(), posHip = Eigen::Vector3d::Zero();

    /// IMU - Initialization data
    _qHip.w() = 0.;
    _qHip.x() = 0.;
    _qHip.y() = 0.;
    _qHip.z() = 0.;
    _qHand.w() = 0.;
    _qHand.x() = 0.;
    _qHand.y() = 0.;
    _qHand.z() = 0.;
    _qTrunk.w() = 0.;
    _qTrunk.x() = 0.;
    _qTrunk.y() = 0.;
    _qTrunk.z() = 0.;
    _qArm.w() = 0.;
    _qArm.x() = 0.;
    _qArm.y() = 0.;
    _qArm.z() = 0.;

    double qWhite[4], qRed[4], qYellow[4];
    if (_robot->sensors.white_imu) {
        _robot->sensors.white_imu->get_quat(qWhite);
        _qHand.w() = qWhite[0];
        _qHand.x() = qWhite[1];
        _qHand.y() = qWhite[2];
        _qHand.z() = qWhite[3];
        //        debug() << "qwhite: " << qWhite[0] << "; " << qWhite[1] << "; " << qWhite[2] << "; " << qWhite[3];
    }
    if (_robot->sensors.red_imu) {
        _robot->sensors.red_imu->get_quat(qRed);
        _qHip.w() = qRed[0];
        _qHip.x() = qRed[1];
        _qHip.y() = qRed[2];
        _qHip.z() = qRed[3];
        //        _qArm.w() = qRed[0];
        //        _qArm.x() = qRed[1];
        //        _qArm.y() = qRed[2];
        //        _qArm.z() = qRed[3];
        //        debug() << "qRed: " << _qHip.w() << "; " << _qHip.x() << "; " << _qHip.y() << "; " << _qHip.z();
    }
    if (_robot->sensors.yellow_imu) {
        _robot->sensors.yellow_imu->get_quat(qYellow);
        _qTrunk.w() = qYellow[0];
        _qTrunk.x() = qYellow[1];
        _qTrunk.y() = qYellow[2];
        _qTrunk.z() = qYellow[3];
        //        debug() << "qyellow: " << qYellow[0] << "; " << qYellow[1] << "; " << qYellow[2] << "; " << qYellow[3];
    }

    /// PIN PUSH-BUTTONS CONTROL
    int pin_down_value = digitalRead(_pin_down);
    int pin_up_value = digitalRead(_pin_up);

    /// CONTROL LOOP
    if (_cnt == 0) {
        initializationLaw(_qHip, period());
    } else if (_cnt <= _init_cnt) {
        initialPositionsLaw(_qHand, _qHip, _qTrunk, _qArm, _theta, _lt, _lsh, _l, _nbDOF, _cnt, _init_cnt);
    } else {
        controlLaw(_qHand, _qHip, _qTrunk, _qArm, _theta, _lt, _lsh, _l, _nbDOF, _k, _lambda, _threshold, _cnt, _init_cnt);

        Eigen::Matrix<double, nbLinks, 1, Eigen::DontAlign> _thetaDot_toSend = _lawJ.returnthetaDot_deg();

        if (_robot->joints.wrist_flexion) {
            _robot->joints.wrist_flexion->set_velocity_safe(-_thetaDot_toSend[0]);
            _robot->joints.wrist_pronation->set_velocity_safe(-_thetaDot_toSend[1]);
            _robot->joints.elbow_flexion->set_velocity_safe(-_thetaDot_toSend[2]);
            if (_cnt % 50 == 0) {
                debug() << "wrist flex vel :" << _thetaDot_toSend[0] << "\n";
                debug() << "pronosup vel :" << _thetaDot_toSend[1] << "\n";
                debug() << "elbow flex vel :" << _thetaDot_toSend[2] << "\n";
            }
        } else {
            if (protoCyb) {
                _robot->joints.wrist_pronation->set_velocity_safe(-_thetaDot_toSend[0]);
                _robot->joints.elbow_flexion->set_velocity_safe(-_thetaDot_toSend[1]);
                if (_cnt % 50 == 0) {
                    debug() << "pronosup vel :" << -_thetaDot_toSend[0] << "\n";
                    debug() << "elbow flex vel :" << -_thetaDot_toSend[1] << "\n";
                }
            } else {
                _robot->joints.wrist_pronation->set_velocity_safe(-_thetaDot_toSend[0]);
                _robot->joints.elbow_flexion->set_velocity_safe(_thetaDot_toSend[1]);
                if (_cnt % 50 == 0) {
                    debug() << "pronosup vel :" << -_thetaDot_toSend[0] << "\n";
                    debug() << "elbow flex vel :" << _thetaDot_toSend[1] << "\n";
                }
            }
        }

        // control with push buttons to mimick myo to control the hand
        //        static int prev_pin_up_value = 1, prev_pin_down_value = 1;
        //                int pin_down_value = _robot->btn2;
        //        int pin_up_value = _robot->btn1;

        //        if (pin_down_value == 0 && prev_pin_down_value == 1) {
        //            _robot->joints.hand->move(TouchBionicsHand::HAND_OPENING_ALL);
        //        } else if (pin_up_value == 0 && prev_pin_up_value == 1) {
        //            _robot->joints.hand->move(TouchBionicsHand::HAND_CLOSING_ALL);
        //        } else if ((pin_down_value == 1 && pin_up_value == 1) && (prev_pin_down_value == 0 || prev_pin_up_value == 0)) {
        //        }
    }

    if (saveData) {
        _lawJ.writeDebugData(debugData, _theta);
        /// WRITE DATA
        _file << _nbDOF << timeWithDelta << ' ' << pin_down_value << ' ' << pin_up_value << ' ' << _lt << ' ' << _lsh << ' ' << _lua << ' ' << _lfa << ' ' << _lwrist;
        _file << ' ' << _lambda[0] << ' ' << _lambda[1] << ' ' << _lambda[2] << ' ' << _threshold[0] << ' ' << _threshold[1] << ' ' << _threshold[2];
        _file << ' ' << qWhite[0] << ' ' << qWhite[1] << ' ' << qWhite[2] << ' ' << qWhite[3] << ' ' << qRed[0] << ' ' << qRed[1] << ' ' << qRed[2] << ' ' << qRed[3];
        _file << ' ' << qYellow[0] << ' ' << qYellow[1] << ' ' << qYellow[2] << ' ' << qYellow[3];
        for (int i = 0; i < 40; i++) {
            _file << ' ' << debugData[i];
        }
        _file << ' ' << pronoSupEncoder << ' ' << wristFlexEncoder << ' ' << elbowEncoder << ' ' << _emg[0] << ' ' << _emg[1];

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

    ++_cnt;
    //    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()).count() << "ms" << std::endl;
}

void ControlIMU::cleanup()
{
    _robot->joints.wrist_pronation->forward(0);
    if (_robot->joints.wrist_flexion)
        _robot->joints.wrist_flexion->forward(0);
    //    _robot->joints.elbow_flexion->move_to(0, 20);
    if (_robot->joints.hand)
        _robot->joints.hand->release_ownership();
    if (saveData)
        _file.close();
}

void ControlIMU::initializationLaw(Eigen::Quaterniond qHi, double p)
{
}

void ControlIMU::initialPositionsLaw(Eigen::Quaterniond qHa, Eigen::Quaterniond qHi, Eigen::Quaterniond qT, Eigen::Quaterniond qA, double theta[], int lt, int lsh, int l[], int nbDOF, int cnt, int init_cnt)
{
}

void ControlIMU::controlLaw(Eigen::Quaterniond qHa, Eigen::Quaterniond qHi, Eigen::Quaterniond qT, Eigen::Quaterniond qA, double theta[], int lt, int lsh, int l[], int nbDOF, int k, double lambda[], double threshold[], int cnt, int init_cnt)
{
}
