
#include "jfoptiorientation.h"
#include "algo/myocontrol.h"
#include "components/internal/hand/touch_bionics_hand.h"
#include "utils/check_ptr.h"
#include "utils/log/log.h"
#include <filesystem>
#include <iostream>

JFOptiOrientation::JFOptiOrientation(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("JFOptitrack Orientation", 0.025)
    , _robot(robot)
    , _k("k", BaseParam::ReadWrite, this, 0.1)
    , _pin_up(24)
    , _pin_down(22)
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

    _menu->set_description("JF Opti Orientation");
    _menu->set_code("jfoo");
    _menu->add_item("tareAll", "Tare all IMUs", [this](std::string) { this->tare_allIMU(); });
    _menu->add_item("tareWhite", "Tare white IMU (trunk)", [this](std::string) { this->tare_whiteIMU(); });
    _menu->add_item("tareYellow", "Tare yellow IMU (hip)", [this](std::string) { this->tare_yellowIMU(); });
    _menu->add_item("tareRed", "Tare red IMU (hand)", [this](std::string) { this->tare_redIMU(); });
    _menu->add_item("calib", "Calibration", [this](std::string) { this->calibrations(); });
    _menu->add_item("elbow90", "Flex elbow at 90 deg", [this](std::string) { this->elbowTo90(); });
    _menu->add_item("pos0", "Flex elbow at 50 deg, rotate wrist to -90 deg", [this](std::string) { this->toPos0(); });
    _menu->add_item("pos1", "Flex elbow at 90 deg, rotate wrist to 30 deg", [this](std::string) { this->toPos1(); });
    _menu->add_item("pos2", "Flex elbow at 100 deg, rotate wrist to -30 deg", [this](std::string) { this->toPos2(); });
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
}

JFOptiOrientation::~JFOptiOrientation()
{
    _robot->joints.elbow_flexion->forward(0);
    _robot->joints.wrist_pronation->forward(0);
    stop_and_join();
}

void JFOptiOrientation::tare_allIMU()
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
            //            debug() << "qWhite: " << qWhite[0] << "; " << qWhite[1] << "; " << qWhite[2] << "; " << qWhite[3];
        }
        if (_robot->sensors.yellow_imu) {
            _robot->sensors.yellow_imu->get_quat(qYellow);
            //            debug() << "qyellow: " << qYellow[0] << "; " << qYellow[1] << "; " << qYellow[2] << "; " << qYellow[3];
        }
        if (_robot->sensors.red_imu) {
            _robot->sensors.red_imu->get_quat(qRed);
            //            debug() << "qred: " << qRed[0] << "; " << qRed[1] << "; " << qRed[2] << "; " << qRed[3];
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

void JFOptiOrientation::tare_whiteIMU()
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

void JFOptiOrientation::tare_yellowIMU()
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

void JFOptiOrientation::tare_redIMU()
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

void JFOptiOrientation::elbowTo90()
{
    if (protoCyb)
        _robot->joints.elbow_flexion->move_to(100, 20);
    else
        _robot->joints.elbow_flexion->move_to(-90, 10);
}

void JFOptiOrientation::toPos0()
{
    if (protoCyb)
        _robot->joints.elbow_flexion->move_to(50, 20);
    else
        _robot->joints.elbow_flexion->move_to(-50, 10);

    _robot->joints.wrist_pronation->move_to(-90, 40);
}

void JFOptiOrientation::toPos1()
{
    if (protoCyb)
        _robot->joints.elbow_flexion->move_to(100, 20);
    else
        _robot->joints.elbow_flexion->move_to(-90, 10);

    _robot->joints.wrist_pronation->move_to(30, 40);
}

void JFOptiOrientation::toPos2()
{
    if (protoCyb)
        _robot->joints.elbow_flexion->move_to(110, 20);
    else
        _robot->joints.elbow_flexion->move_to(-100, 10);

    _robot->joints.wrist_pronation->move_to(-30, 40);
}

void JFOptiOrientation::set_velocity_motors(double speed_elbow, double speed_wrist)
{
    // ELBOW
    if (_robot->joints.elbow_flexion->is_calibrated() == false) {
        warning() << "Elbow not calibrated...";
        return;
    }
    int32_t pos_elbow;
    double max_angle_elbow = _robot->joints.elbow_flexion->get_max_angle();
    double min_angle_elbow = _robot->joints.elbow_flexion->get_min_angle();
    if (speed_elbow > 0)
        pos_elbow = static_cast<int32_t>(max_angle_elbow * _robot->joints.elbow_flexion->r_incs_per_deg());
    else
        pos_elbow = static_cast<int32_t>(min_angle_elbow * _robot->joints.elbow_flexion->r_incs_per_deg());

    uint32_t velocity_elbow = static_cast<uint32_t>(std::fabs(speed_elbow) * _robot->joints.elbow_flexion->r_incs_per_deg());
    if (velocity_elbow == 0)
        velocity_elbow = 1;
    uint32_t acc_elbow = _robot->joints.elbow_flexion->get_acc();

    // WRIST PRONATION
    int32_t pos_wrist;
    int32_t min_angle_wrist = -130;
    int32_t max_angle_wrist = 90;
    if (speed_wrist > 0)
        pos_wrist = max_angle_wrist * static_cast<int32_t>(_robot->joints.wrist_pronation->r_incs_per_deg());
    else
        pos_wrist = min_angle_wrist * static_cast<int32_t>(_robot->joints.wrist_pronation->r_incs_per_deg());

    uint32_t velocity_wrist = static_cast<uint32_t>(std::fabs(speed_wrist) * _robot->joints.wrist_pronation->r_incs_per_deg());
    if (velocity_wrist == 0)
        velocity_wrist = 1;
    uint32_t acc_wrist = 10 * _robot->joints.wrist_pronation->get_acc();

    // SEND COMMAND
    try {
        _robot->joints.elbow_flexion->move_to_2motors(acc_elbow, velocity_elbow, acc_elbow, pos_elbow, acc_wrist, velocity_wrist, acc_wrist, pos_wrist);
    } catch (std::runtime_error& e) {
        std::cout << "Runtime error when sending velocities : " << e.what() << std::endl;
    }
}

void JFOptiOrientation::calibrations()
{
    // HAND
    if (_robot->joints.hand) {
        _robot->joints.hand->take_ownership();
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
        _robot->joints.elbow_flexion->move_to(100, 20, true);
    else
        _robot->joints.elbow_flexion->move_to(-90, 20);
}

void JFOptiOrientation::displayPin()
{
    int pin_down_value = _robot->btn2;
    int pin_up_value = _robot->btn1;
    debug() << "PinUp: " << pin_up_value;
    debug() << "PinDown: " << pin_down_value;
}

void JFOptiOrientation::displayRBnb()
{
    _robot->sensors.optitrack->update();
    optitrack_data_t data = _robot->sensors.optitrack->get_last_data();
    debug() << "nbRigid Bodies: " << data.nRigidBodies << "\n";
    std::cout << "nbRigid Bodies: " << data.nRigidBodies << std::endl;
}

bool JFOptiOrientation::setup()
{
    _robot->joints.wrist_pronation->set_encoder_position(0);

    // Check for OptiTrack
    _robot->sensors.optitrack->update();
    optitrack_data_t data = _robot->sensors.optitrack->get_last_data();
    if (data.nRigidBodies == 0 || data.nRigidBodies > 100) {
        critical() << "No data from OptiTrack"
                   << "\n";
        std::cout << "No data from OptiTrack" << std::endl;
        _robot->user_feedback.buzzer->makeNoise(Buzzer::ERROR_BUZZ);
        return false;
    }

    // OPEN AND NAME DATA FILE
    if (saveData) {
        std::string filename("JFOptiOrientation");
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

void JFOptiOrientation::loop(double, clock::time_point time)
{
    static double pronoSupEncoder = 0;
    static double elbowEncoder = 0;
    static double wristFlexEncoder = 0;
    static double tmpEncoder = 0;

    int init_cnt = 10;
    double timeWithDelta = (time - _start_time).count();
    std::cout << timeWithDelta << std::endl;

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

    /// WRIST
    try {
        tmpEncoder = pronoSupEncoder;
        pronoSupEncoder = _robot->joints.wrist_pronation->read_encoder_position();
    } catch (std::runtime_error& e) {
        std::cout << "Runtime error when reading wrist encoder: " << e.what() << std::endl;
        pronoSupEncoder = tmpEncoder;
    }

    ///GET DATA
    /// OPTITRACK
    int index_acromion = -1, index_hip = -1, index_hand = -1;
    Eigen::Vector3d posA = Eigen::Vector3d::Zero(), posHip = Eigen::Vector3d::Zero();
    Eigen::Quaterniond qHip;
    qHip.w() = 0.;
    qHip.x() = 0.;
    qHip.y() = 0.;
    qHip.z() = 0.;
    Eigen::Quaterniond qHipOpti;
    qHipOpti.w() = 0.;
    qHipOpti.x() = 0.;
    qHipOpti.y() = 0.;
    qHipOpti.z() = 0.;
    Eigen::Quaterniond qHand;
    qHand.w() = 0.;
    qHand.x() = 0.;
    qHand.y() = 0.;
    qHand.z() = 0.;
    Eigen::Quaterniond qHandOpti;
    qHandOpti.w() = 0.;
    qHandOpti.x() = 0.;
    qHandOpti.y() = 0.;
    qHandOpti.z() = 0.;
    Eigen::Quaterniond qTrunk;
    qTrunk.w() = 0.;
    qTrunk.x() = 0.;
    qTrunk.y() = 0.;
    qTrunk.z() = 0.;

    for (int i = 0; i < nbRigidBodies; i++) {
        if (data.rigidBodies[i].ID == 4) {
            // acromion position
            posA[0] = data.rigidBodies[i].x * 100;
            posA[1] = data.rigidBodies[i].y * 100;
            posA[2] = data.rigidBodies[i].z * 100;
            index_acromion = i;
        } else if (data.rigidBodies[i].ID == 7) {
            // hip position and orientation
            posHip[0] = data.rigidBodies[i].x * 100;
            posHip[1] = data.rigidBodies[i].y * 100;
            posHip[2] = data.rigidBodies[i].z * 100;
            qHipOpti.w() = data.rigidBodies[i].qw;
            qHipOpti.x() = data.rigidBodies[i].qx;
            qHipOpti.y() = data.rigidBodies[i].qy;
            qHipOpti.z() = data.rigidBodies[i].qz;
            index_hip = i;
        } else if (data.rigidBodies[i].ID == 1) {
            // hand orientation
            qHandOpti.w() = data.rigidBodies[i].qw;
            qHandOpti.x() = data.rigidBodies[i].qx;
            qHandOpti.y() = data.rigidBodies[i].qy;
            qHandOpti.z() = data.rigidBodies[i].qz;
            index_hand = i;
        }
    }
    //    if (_cnt % 50 == 0) {
    //        debug() << "posA: " << posA[0] << "; " << posA[1] << "; " << posA[2];
    //    }

    /// ELBOW
    try {
        tmpEncoder = elbowEncoder;
        elbowEncoder = _robot->joints.elbow_flexion->read_encoder_position();
    } catch (std::runtime_error& e) {
        std::cout << "Runtime error when reading elbow encoder: " << e.what() << std::endl;
        elbowEncoder = tmpEncoder;
    }

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

        wristFlexEncoder = _robot->joints.wrist_flexion->read_encoder_position();
        theta[0] = -wristFlexEncoder / _robot->joints.wrist_flexion->r_incs_per_deg();

        pronoSupEncoder = _robot->joints.wrist_pronation->read_encoder_position();
        theta[1] = pronoSupEncoder / _robot->joints.wrist_pronation->r_incs_per_deg();

        theta[2] = -elbowEncoder / _robot->joints.elbow_flexion->r_incs_per_deg();
        //        if (_cnt % 50 == 0)
        //            debug() << "theta(deg): " << theta[0] << ", " << theta[1] << ", " << theta[2] << "\r\n";
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

        if (protoCyb) {
            theta[1] = -elbowEncoder / _robot->joints.elbow_flexion->r_incs_per_deg();
            theta[0] = -pronoSupEncoder / _robot->joints.wrist_pronation->r_incs_per_deg();
        } else {
            theta[1] = elbowEncoder / _robot->joints.elbow_flexion->r_incs_per_deg();
            theta[0] = -pronoSupEncoder / _robot->joints.wrist_pronation->r_incs_per_deg();
        }

        //        double eRed[2];
        //        _robot->sensors.red_imu->get_euler(eRed);
        //        _mqtt.publish("sam/emg/time/0", std::to_string(-theta[0]));
        //        _mqtt.publish("sam/emg/time/1", std::to_string(eRed[0]));

        if (_cnt % 50 == 0)
            debug() << "theta(deg): " << theta[0] << ", " << theta[1] << "\r\n";
        // in radians:
        theta[0] = theta[0] * M_PI / 180;
        theta[1] = theta[1] * M_PI / 180;
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
        qTrunk.w() = qWhite[0];
        qTrunk.x() = qWhite[1];
        qTrunk.y() = qWhite[2];
        qTrunk.z() = qWhite[3];
    }

    /// CONTROL LOOP
    if (_cnt == 0) {
        _lawJ.initialization(qHip, 1 / period());
        _lawJ.initializationOpti(posA);
    } else if (_cnt <= init_cnt) {
        _lawJ.initialPositions(posA, posHip, _cnt, init_cnt);
        _lawJ.rotationMatrices2(qHandOpti, qHand, qHipOpti, qHip, qTrunk);
        _lawJ.updateFrames(theta);
        _lawJ.orientationInWrist(posA, posHip, _cnt, init_cnt);
    } else {
        _lawJ.rotationMatrices2(qHandOpti, qHand, qHipOpti, qHip, qTrunk);
        //        _lawJ.orientationInHand(posA, posHip, _cnt, init_cnt);
        _lawJ.updateFrames(theta);
        _lawJ.orientationInWrist(posA, posHip, _cnt, init_cnt);
        _lawJ.controlLaw_orientation(_k, _lambda, _threshold, _cnt);

        Eigen::Matrix<double, nbLinks, 1, Eigen::DontAlign> thetaDot_toSend = _lawJ.returnthetaDot_deg();

        if (_robot->joints.wrist_flexion) {
            //            _robot->joints.wrist_flexion->set_velocity_safe(-thetaDot_toSend[0]);
            //            _robot->joints.wrist_pronation->set_velocity_safe(thetaDot_toSend[1]);
            //            _robot->joints.elbow_flexion->set_velocity_safe(-thetaDot_toSend[2]);
            //            if (_cnt % 50 == 0) {
            //                debug() << "wrist flex vel :" << thetaDot_toSend[0] << "\n";
            //                debug() << "pronosup vel :" << thetaDot_toSend[1] << "\n";
            //                debug() << "elbow flex vel :" << thetaDot_toSend[2] << "\n";
            //            }
        } else {

            if (protoCyb) {
                set_velocity_motors(-thetaDot_toSend[1], -thetaDot_toSend[0]);
                //                if (_cnt % 50 == 0) {
                //                    debug() << "pronosup vel :" << -thetaDot_toSend[0] << "\n";
                //                    debug() << "elbow flex vel :" << -thetaDot_toSend[1] << "\n";
                //                }
            } else {
                set_velocity_motors(thetaDot_toSend[1], -thetaDot_toSend[0]);
                if (_cnt % 50 == 0) {
                    debug() << "pronosup vel :" << -thetaDot_toSend[0] << "\n";
                    debug() << "elbow flex vel :" << thetaDot_toSend[1] << "\n";
                }
            }
        }
    }

    /// PUSH-BUTTONS FOR HAND CONTROL
    int16_t electrodes[2];
    int pin_down_value = 0, pin_up_value = 0;
    if (protoCyb) {
        //EMG5 and EMG6: push-buttons
        electrodes[0] = _robot->sensors.adc3->readADC_SingleEnded(2);
        electrodes[1] = _robot->sensors.adc3->readADC_SingleEnded(0);
        if (_cnt % 50 == 0) {
            debug() << "Electrodes 5-6: " << electrodes[0] << "; " << electrodes[1];
        }

        if (electrodes[1] <= 0 && electrodes[0] > 0) {
            // open hand
            _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 2, 2);
        } else if (electrodes[0] <= 0 && electrodes[1] > 0) {
            //close hand
            _robot->joints.hand_quantum->makeContraction(QuantumHand::SHORT_CONTRACTION, 1, 2);
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

    if (saveData) {
        _lawJ.writeDebugData(debugData, theta);
        /// WRITE DATA
        if (protoCyb)
            _file << nbDOF << ' ' << timeWithDelta << ' ' << electrodes[0] << ' ' << electrodes[1];
        else
            _file << nbDOF << ' ' << timeWithDelta << ' ' << pin_down_value << ' ' << pin_up_value;

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
    //std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()).count() << "ms" << std::endl;
}

void JFOptiOrientation::cleanup()
{
    _robot->joints.wrist_pronation->forward(0);
    if (_robot->joints.wrist_flexion)
        _robot->joints.wrist_flexion->forward(0);
    _robot->joints.elbow_flexion->set_velocity_safe(0);
    if (_robot->joints.hand)
        _robot->joints.hand->release_ownership();
    _file.close();
}
