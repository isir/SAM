#include "jf_opti.h"
#include "algo/myocontrol.h"
#include "components/internal/hand/touch_bionics_hand.h"
#include "utils/check_ptr.h"
#include "utils/log/log.h"
#include <filesystem>
#include <iostream>

JacobianFormulationOpti::JacobianFormulationOpti(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("Jacobian Formulation Optitrack", 0.01)
    , _robot(robot)
    , _k("k", BaseParam::ReadWrite, this, 5)
    , _lt(40)
    , _pin_up(24)
    , _pin_down(22)
    , _lua("lua(cm)", BaseParam::ReadWrite, this, 30)
    , _lfa("lfa(cm)", BaseParam::ReadWrite, this, 20)
    , _lwrist("lwrist(cm)", BaseParam::ReadWrite, this, 10)
    , _lambdaE("lambda elbow", BaseParam::ReadWrite, this, 2)
    , _lambdaWF("lambda wrist flex", BaseParam::ReadWrite, this, 0)
    , _lambdaWPS("lambda wrist PS", BaseParam::ReadWrite, this, 4)
    , _thresholdE("threshold E", BaseParam::ReadWrite, this, 5)
    , _thresholdWF("threshold WF", BaseParam::ReadWrite, this, 5)
    , _thresholdWPS("threshold WPS", BaseParam::ReadWrite, this, 5)
{
    if (!check_ptr(_robot->joints.elbow_flexion, _robot->joints.wrist_pronation, _robot->sensors.optitrack)) {
        throw std::runtime_error("Jacobian Formulation Control is missing components");
    }

    _menu->set_description("Jacobian Formulation Opti");
    _menu->set_code("jfo");
    _menu->add_item("tareAll", "Tare all IMUs", [this](std::string) { this->tare_allIMU(); });
    _menu->add_item("tareWhite", "Tare white IMU", [this](std::string) { this->tare_whiteIMU(); });
    _menu->add_item("tareYellow", "Tare yellow IMU", [this](std::string) { this->tare_yellowIMU(); });
    _menu->add_item("tareRed", "Tare red IMU", [this](std::string) { this->tare_redIMU(); });
    _menu->add_item("calib", "Calibration", [this](std::string) { this->calibrations(); });
    _menu->add_item("elbow90", "Flex elbow at 90 deg", [this](std::string) { this->elbowTo90(); });
    _menu->add_item("opti", "Check OptiTrack", [this](std::string) { this->displayRBnb(); });

    _menu->add_item(_robot->joints.elbow_flexion->menu());
    _menu->add_item(_robot->joints.wrist_pronation->menu());
    if (_robot->joints.hand)
        _menu->add_item(_robot->joints.hand->menu());

    if (_robot->joints.wrist_flexion) {
        _menu->add_item(_robot->joints.wrist_flexion->menu());

        nbDOF = 3; // set the number of dof of the prosthetic arm to adapt intermediate computations in control law
        debug() << "nDOF: " << nbDOF;
    } else {
        nbDOF = 2;
        debug() << "nDOF: " << nbDOF;
    }

    //    for (int i = 0; i < nbLinks; i++) {
    //        _threshold[i] = 10. * M_PI / 180;
    //    }
}

JacobianFormulationOpti::~JacobianFormulationOpti()
{
    _robot->joints.elbow_flexion->forward(0);
    _robot->joints.wrist_pronation->forward(0);
    stop_and_join();
}

void JacobianFormulationOpti::tare_allIMU()
{
    if (!_robot->sensors.red_imu || !_robot->sensors.yellow_imu) {
        std::cout << "An IMU is missing."  << std::endl;
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

        double qWhite[4], qYellow[4], qRed[4], qNG[4];
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

        if (qRed[0]<5E-5 || qYellow[0]<5E-5) {
            std::cout << "An IMU was not correctly tared."  << std::endl;
            critical() << "An IMU was not correctly tared.";
            _robot->user_feedback.buzzer->makeNoise(Buzzer::ERROR_BUZZ);
        } else {
            _robot->user_feedback.buzzer->makeNoise(Buzzer::TRIPLE_BUZZ);
        }
    }
}

void JacobianFormulationOpti::tare_whiteIMU()
{
    if (!_robot->sensors.white_imu) {
        std::cout << "White IMU is missing."  << std::endl;
        critical() << "White IMU is missing.";
        _robot->user_feedback.buzzer->makeNoise(Buzzer::ERROR_BUZZ);
    } else {

        _robot->sensors.white_imu->send_command_algorithm_init_then_tare();

        debug("Wait ...");

        usleep(6 * 1000000);

        double qWhite[4];
        _robot->sensors.white_imu->get_quat(qWhite);
        debug() << "qWhite: " << qWhite[0] << "; " << qWhite[1] << "; " << qWhite[2] << "; " << qWhite[3];

        if (qWhite[0]<5E-5) {
            std::cout << "White IMU was not correctly tared."  << std::endl;
            critical() << "White IMU was not correctly tared.";
            _robot->user_feedback.buzzer->makeNoise(Buzzer::ERROR_BUZZ);
        } else {
            _robot->user_feedback.buzzer->makeNoise(Buzzer::TRIPLE_BUZZ);
        }
    }
}

void JacobianFormulationOpti::tare_yellowIMU()
{
    if (!_robot->sensors.yellow_imu) {
        std::cout << "Yellow IMU is missing."  << std::endl;
        critical() << "Yellow IMU is missing.";
        _robot->user_feedback.buzzer->makeNoise(Buzzer::ERROR_BUZZ);
    } else {
        _robot->sensors.yellow_imu->send_command_algorithm_init_then_tare();

        debug("Wait ...");

        usleep(6 * 1000000);

        double qYellow[4];
        _robot->sensors.yellow_imu->get_quat(qYellow);
        debug() << "qyellow: " << qYellow[0] << "; " << qYellow[1] << "; " << qYellow[2] << "; " << qYellow[3];

        if (qYellow[0]<5E-5) {
            std::cout << "Yellow IMU was not correctly tared."  << std::endl;
            critical() << "Yellow IMU was not correctly tared.";
            _robot->user_feedback.buzzer->makeNoise(Buzzer::ERROR_BUZZ);
        } else {
            _robot->user_feedback.buzzer->makeNoise(Buzzer::TRIPLE_BUZZ);
        }
    }
}

void JacobianFormulationOpti::tare_redIMU()
{
    if (!_robot->sensors.red_imu) {
        std::cout << "Red IMU is missing."  << std::endl;
        critical() << "Red IMU is missing.";
        _robot->user_feedback.buzzer->makeNoise(Buzzer::ERROR_BUZZ);
    } else {
        _robot->sensors.red_imu->send_command_algorithm_init_then_tare();

        debug("Wait ...");

        usleep(6 * 1000000);

        double qRed[4];
        _robot->sensors.red_imu->get_quat(qRed);
        debug() << "qred: " << qRed[0] << "; " << qRed[1] << "; " << qRed[2] << "; " << qRed[3];

        if (qRed[0]<5E-5) {
            std::cout << "Red IMU was not correctly tared."  << std::endl;
            critical() << "Red IMU was not correctly tared.";
            _robot->user_feedback.buzzer->makeNoise(Buzzer::ERROR_BUZZ);
        } else {
            _robot->user_feedback.buzzer->makeNoise(Buzzer::TRIPLE_BUZZ);
        }
    }
}

void JacobianFormulationOpti::elbowTo90()
{
    if (protoCyb)
        _robot->joints.elbow_flexion->move_to(90, 10);
    else
        _robot->joints.elbow_flexion->move_to(-90, 10);
}

void JacobianFormulationOpti::calibrations()
{
    // HAND
    if (_robot->joints.hand) {
        _robot->joints.hand->take_ownership();
//        _robot->joints.hand->init_sequence();
//        _robot->joints.hand->setPosture(TouchBionicsHand::PINCH_POSTURE);
//        _robot->joints.hand->move(TouchBionicsHand::PINCH_OPENING);
    }

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
        _robot->joints.elbow_flexion->move_to(90, 30, true);
    else
        _robot->joints.elbow_flexion->move_to(-90, 20);
}

void JacobianFormulationOpti::receiveData()
{
    while (_receiver.available()) {

        auto data = _receiver.receive();
        std::string buf;
        buf.resize(data.size());
        std::transform(data.begin(), data.end(), buf.begin(), [](std::byte b) -> char { return static_cast<char>(b); });
        std::istringstream ts(buf);
        int tmp;

        //        ts >> tmp;
        //        _lua = tmp; // _lua

        //        ts >> tmp;
        //        _lfa = tmp; // _lfa

        //        ts >> tmp;
        //        _lwrist = tmp; // _lwrist

        //        ts >> tmp;
        //        _lhand = tmp; // _lhand

        //        ts >> tmp;
        //        _lambda = tmp;

        ts >> tmp;
        _threshold[0] = tmp * M_PI / 180.; // dead zone limit for pronosup, in rad.

        ts >> tmp;
        _threshold[1] = tmp * M_PI / 180; // dead zone limit for wrist flex, in rad.

        ts >> tmp;
        _threshold[2] = tmp * M_PI / 180; // dead zone limit for elbow flex, in rad.
    }
}

void JacobianFormulationOpti::displayPin()
{
    int pin_down_value = _robot->btn2;
    int pin_up_value = _robot->btn1;
    debug() << "PinUp: " << pin_up_value;
    debug() << "PinDown: " << pin_down_value;
}

void JacobianFormulationOpti::displayRBnb()
{
    _robot->sensors.optitrack->update();
    optitrack_data_t data = _robot->sensors.optitrack->get_last_data();
    debug() << "nbRigid Bodies: " << data.nRigidBodies << "\n";
    std::cout << "nbRigid Bodies: " << data.nRigidBodies << std::endl;
}

bool JacobianFormulationOpti::setup()
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

    _robot->joints.wrist_pronation->set_encoder_position(0);

    // Check for OptiTrack
    _robot->sensors.optitrack->update();
    optitrack_data_t data = _robot->sensors.optitrack->get_last_data();
    if (data.nRigidBodies==0 || data.nRigidBodies>100) {
        critical() << "No data from OptiTrack" << "\n";
        std::cout << "No data from OptiTrack" << std::endl;
        _robot->user_feedback.buzzer->makeNoise(Buzzer::ERROR_BUZZ);
        return false;
    }


    // OPEN AND NAME DATA FILE
    if (saveData) {
        std::string filename("JFOpti");
        std::string suffix;

        int cnt = 0;
        nbRigidBodies = 0;
        std::string extension(".txt");
        do {
            ++cnt;
            suffix = "_" + std::to_string(cnt);
        } while (std::filesystem::exists(filename + suffix + extension));

        _file = std::ofstream(filename + suffix + extension);
        if (!_file.good()) {
            critical() << "Failed to open" << (filename + suffix + extension);
            _robot->user_feedback.buzzer->makeNoise(Buzzer::ERROR_BUZZ);
            return false;
        }
        _need_to_write_header = true;
    }
    _start_time = clock::now();

    // INITIALIZATION
    _cnt = 0;
    for (int i = 0; i < nbLinks; i++) {
        theta[i] = 0.;
    }
    return true;
}

void JacobianFormulationOpti::loop(double, clock::time_point time)
{
    int init_cnt = 10;
    double timeWithDelta = (time - _start_time).count();

    static std::unique_ptr<MyoControl::Classifier> handcontrol;

    static const unsigned int counts_after_mode_change = 15;
    static const unsigned int counts_btn = 2;
    static const unsigned int counts_before_bubble = 2;
    static const unsigned int counts_after_bubble = 10;
    static const MyoControl::EMGThresholds thresholds(15, 8, 15, 15, 8, 15);

    _robot->sensors.optitrack->update();
    optitrack_data_t data = _robot->sensors.optitrack->get_last_data();
    if (_cnt == 0) {
        debug() << "nbRigid Bodies: " << data.nRigidBodies << "\n";
        if (data.nRigidBodies < 10)
            nbRigidBodies = data.nRigidBodies;
    }

    double debugData[40];

    /// WRITE FILE HEADERS
    if (saveData) {
        if (_need_to_write_header) {
            _file << " time, pinUp, pinDown,";
            _file << " qWhite.w, qWhite.x, qWhite.y, qWhite.z, qTronc.w, qTronc.x, qTronc.y, qTronc.z,";
            _file << " qYellow.w, qYellow.x, qYellow.y, qYellow.z,";
            _file << " to complete,";
            _file << " nbRigidBodies";
            for (int i = 0; i < nbRigidBodies; i++) {
                _file << ", ID, bTrackingValid, fError, qw, qx, qy, qz, x, y, z";
            }
            _file << "\r\n";
            _need_to_write_header = false;
        }
    }

    // button "mode compensation" of cybathlon to indicate beginning of motion
    int btnStart;
    if (!_robot->btn3)
        btnStart = 0;
    else
        btnStart = 1;

    ///GET DATA
    /// OPTITRACK
    int index_acromion = -1, index_hip = -1, index_hand = -1;
    Eigen::Vector3d posA = Eigen::Vector3d::Zero(), posHip = Eigen::Vector3d::Zero();
    Eigen::Quaterniond qHip;
    qHip.w() = 0.;
    qHip.x() = 0.;
    qHip.y() = 0.;
    qHip.z() = 0.;
    Eigen::Quaterniond qHand;
    qHand.w() = 0.;
    qHand.x() = 0.;
    qHand.y() = 0.;
    qHand.z() = 0.;
    Eigen::Quaterniond qTrunk;
    qTrunk.w() = 0.;
    qTrunk.x() = 0.;
    qTrunk.y() = 0.;
    qTrunk.z() = 0.;

    Eigen::Quaterniond qArm;
    qArm.w() = 0.;
    qArm.x() = 0.;
    qArm.y() = 0.;
    qArm.z() = 0.;

    for (int i = 0; i < nbRigidBodies; i++) {
        if (data.rigidBodies[i].ID == 4) {
            posA[0] = data.rigidBodies[i].x * 100;
            posA[1] = data.rigidBodies[i].y * 100;
            posA[2] = data.rigidBodies[i].z * 100;
            index_acromion = i;
        } else if (data.rigidBodies[i].ID == 7) {
            posHip[0] = data.rigidBodies[i].x * 100;
            posHip[1] = data.rigidBodies[i].y * 100;
            posHip[2] = data.rigidBodies[i].z * 100;
            //            qHip.w() = data.rigidBodies[i].qw;
            //            qHip.x() = data.rigidBodies[i].qx;
            //            qHip.y() = data.rigidBodies[i].qy;
            //            qHip.z() = data.rigidBodies[i].qz;
            index_hip = i;
        } else if (data.rigidBodies[i].ID == 2) {
            //            qHand.w() = data.rigidBodies[i].qw;
            //            qHand.x() = data.rigidBodies[i].qx;
            //            qHand.y() = data.rigidBodies[i].qy;
            //            qHand.z() = data.rigidBodies[i].qz;
            index_hand = i;
        }
    }
    //    if (_cnt % 50 == 0) {
    //        debug() << "posA: " << posA[0] << "; " << posA[1] << "; " << posA[2];
    //    }

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
        l[0] = _lwrist; //  from the pronosupination joint to the wrist flexion/ext joint
        l[1] = _lfa; //  from the wrist flex/ext joint to the elbow flex/ext joint
        l[2] = _lua; // from the elbow joint to the acromion/shoulder joint

        wristFlexEncoder = _robot->joints.wrist_flexion->read_encoder_position();
        theta[0] = -wristFlexEncoder / _robot->joints.wrist_flexion->r_incs_per_deg();
        theta[1] = pronoSupEncoder / _robot->joints.wrist_pronation->r_incs_per_deg();
        theta[2] = -elbowEncoder / _robot->joints.elbow_flexion->r_incs_per_deg();
        if (_cnt % 50 == 0)
            debug() << "theta(deg): " << theta[0] << ", " << theta[1] << ", " << theta[2] << "\r\n";
        theta[0] = -M_PI / 2 + theta[0] * M_PI / 180;
        theta[1] = M_PI / 2 + theta[1] * M_PI / 180;
        theta[2] = M_PI / 2 + theta[2] * M_PI / 180;

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
        l[0] = _lwrist;
        l[1] = _lfa;
        l[2] = _lua;

        theta[0] = -pronoSupEncoder / _robot->joints.wrist_pronation->r_incs_per_deg();
        if (protoCyb)
            theta[1] = -elbowEncoder / _robot->joints.elbow_flexion->r_incs_per_deg();
        else
            theta[1] = elbowEncoder / _robot->joints.elbow_flexion->r_incs_per_deg();

        if (_cnt % 50 == 0)
            debug() << "theta(deg): " << theta[0] << ", " << theta[1] << "\r\n";
        // in radians:
        theta[0] = theta[0] * M_PI / 180;
        theta[1] = theta[1] * M_PI / 180;

        _lambda[0] = _lambdaWPS;
        _lambda[1] = _lambdaE;
    }

    /// IMU
    double qWhite[4], qYellow[4], qRed[4];
    if (_robot->sensors.red_imu) {
        _robot->sensors.red_imu->get_quat(qRed);
        qHand.w() = qRed[0];
        qHand.x() = qRed[1];
        qHand.y() = qRed[2];
        qHand.z() = qRed[3];
    }
    if (_robot->sensors.yellow_imu) {
        _robot->sensors.yellow_imu->get_quat(qYellow);
        qHip.w() = qYellow[0];
        qHip.x() = qYellow[1];
        qHip.y() = qYellow[2];
        qHip.z() = qYellow[3];
    }
    if (_robot->sensors.white_imu) {
        _robot->sensors.white_imu->get_quat(qWhite);
        qArm.w() = qWhite[0];
        qArm.x() = qWhite[1];
        qArm.y() = qWhite[2];
        qArm.z() = qWhite[3];
    }

    /// PUSH-BUTTONS FOR HAND CONTROL

    int16_t electrodes[2];
    int pin_down_value, pin_up_value;
    if (protoCyb) {
        //EMG5 and EMG6: push-buttons
        static bool btn = 0;
        electrodes[0] = _robot->sensors.adc3->readADC_SingleEnded(2);
        electrodes[1] = _robot->sensors.adc3->readADC_SingleEnded(0);
        if (_cnt % 50 == 0) {
            debug() << "Electrodes 5-6: " << electrodes[0] << "; " << electrodes[1];
        }

        if (electrodes[0] <= 0 && electrodes[1] > 0) {
            // open hand
            _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 1, 2);
        } else if (electrodes[1] <= 0 && electrodes[0] > 0) {
            //close hand
            _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 2, 2);
        } else {
            _robot->joints.hand_quantum->makeContraction(QuantumHand::STOP);
        }
    } else {

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
        }

        else {
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

    /// CONTROL LOOP
    if (_cnt == 0) {
        _lawJ.initialization(qHip, 1 / period());
        _lawJ.initializationOpti(posA);
    } else if (_cnt <= init_cnt) {
        _lawJ.initialPositions(posA, posHip, _cnt, init_cnt);
        _lawJ.initialQuat(qHip, qTrunk, qArm, _cnt, init_cnt);
        _lawJ.rotationMatrices(qHand, qHip, qTrunk);
        _lawJ.projectionInHip(posA, posHip, _cnt, init_cnt);
        _lawJ.updateFrames(theta);
        _lawJ.computeOriginsVectors(l, nbDOF);
    } else {
        _lawJ.rotationMatrices(qHand, qHip, qTrunk);
        _lawJ.projectionInHip(posA, posHip, _cnt, init_cnt);
        _lawJ.updateFrames(theta);
        _lawJ.computeOriginsVectors(l, nbDOF);
        _lawJ.scaleDisplacementHip(_cnt);
        _lawJ.controlLaw_v1(posA, _k, _lambda, _threshold, _cnt);

        Eigen::Matrix<double, nbLinks, 1, Eigen::DontAlign> thetaDot_toSend = _lawJ.returnthetaDot_deg();

        if (_robot->joints.wrist_flexion) {
            //            _robot->joints.wrist_flexion->set_velocity_safe(-thetaDot_toSend[0]);
            //            _robot->joints.wrist_pronation->set_velocity_safe(thetaDot_toSend[1]);
            //            _robot->joints.elbow_flexion->set_velocity_safe(-thetaDot_toSend[2]);
            if (_cnt % 50 == 0) {
                debug() << "wrist flex vel :" << thetaDot_toSend[0] << "\n";
                debug() << "pronosup vel :" << thetaDot_toSend[1] << "\n";
                debug() << "elbow flex vel :" << thetaDot_toSend[2] << "\n";
            }
        } else {
            if (protoCyb) {
                if ((_robot->joints.wrist_pronation->pos() > 100) && (-thetaDot_toSend[0] > 0))
                    _robot->joints.wrist_pronation->set_velocity_safe(0);
                else if ((_robot->joints.wrist_pronation->pos() < -100) && (-thetaDot_toSend[0] < 0))
                    _robot->joints.wrist_pronation->set_velocity_safe(0);
                else
                    _robot->joints.wrist_pronation->set_velocity_safe(-thetaDot_toSend[0]);

                _robot->joints.elbow_flexion->set_velocity_safe(-thetaDot_toSend[1]);
                if (_cnt % 50 == 0) {
                    debug() << "pronosup vel :" << -thetaDot_toSend[0] << "\n";
                    debug() << "elbow flex vel :" << -thetaDot_toSend[1] << "\n";
                }
            } else {
                if ((_robot->joints.wrist_pronation->pos() > 100) && (-thetaDot_toSend[0] > 0))
                    _robot->joints.wrist_pronation->set_velocity_safe(0);
                else if ((_robot->joints.wrist_pronation->pos() < -100) && (-thetaDot_toSend[0] < 0))
                    _robot->joints.wrist_pronation->set_velocity_safe(0);
                else
                    _robot->joints.wrist_pronation->set_velocity_safe(-thetaDot_toSend[0]);

                _robot->joints.elbow_flexion->set_velocity_safe(thetaDot_toSend[1]);
                if (_cnt % 50 == 0) {
                    debug() << "pronosup vel :" << -thetaDot_toSend[0] << "\n";
                    debug() << "elbow flex vel :" << thetaDot_toSend[1] << "\n";
                }
            }
        }
    }

    if (saveData) {
        _lawJ.writeDebugData(debugData, theta);
        /// WRITE DATA
        if (protoCyb)
            _file << nbDOF << ' ' << timeWithDelta << ' ' << btnStart << ' ' << electrodes[0] << ' ' << electrodes[1] << ' ' << _lua << ' ' << _lfa << ' ' << _lwrist;
        else
            _file << nbDOF << ' ' << timeWithDelta << ' ' << btnStart << ' ' << pin_down_value << ' ' << pin_up_value << ' ' << _lua << ' ' << _lfa << ' ' << _lwrist;

        _file << ' ' << _lambda[0] << ' ' << _lambda[1] << ' ' << _lambda[2] << ' ' << _threshold[0] << ' ' << _threshold[1] << ' ' << _threshold[2];
        _file << ' ' << qRed[0] << ' ' << qRed[1] << ' ' << qRed[2] << ' ' << qRed[3] << ' ' << qWhite[0] << ' ' << qWhite[1] << ' ' << qWhite[2] << ' ' << qWhite[3];
        _file << ' ' << qYellow[0] << ' ' << qYellow[1] << ' ' << qYellow[2] << ' ' << qYellow[3];
        for (int i = 0; i < 40; i++) {
            _file << ' ' << debugData[i];
        }
        _file << ' ' << pronoSupEncoder << ' ' << wristFlexEncoder << ' ' << elbowEncoder;
        _file << ' ' << data.nRigidBodies;

        for (int i = 0; i < nbRigidBodies; i++) {
            _file << ' ' << data.rigidBodies[i].ID << ' ' << data.rigidBodies[i].bTrackingValid << ' ' << data.rigidBodies[i].fError;
            _file << ' ' << data.rigidBodies[i].qw << ' ' << data.rigidBodies[i].qx << ' ' << data.rigidBodies[i].qy << ' ' << data.rigidBodies[i].qz;
            _file << ' ' << data.rigidBodies[i].x << ' ' << data.rigidBodies[i].y << ' ' << data.rigidBodies[i].z;
        }
        _file << std::endl;
    }
    ++_cnt;
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()).count() << "ms" << std::endl;
}

void JacobianFormulationOpti::cleanup()
{
    _robot->joints.wrist_pronation->forward(0);
    if (_robot->joints.wrist_flexion)
        _robot->joints.wrist_flexion->forward(0);
    _robot->joints.elbow_flexion->set_velocity_safe(0);
    if (_robot->joints.hand)
        _robot->joints.hand->release_ownership();
    _file.close();
}
