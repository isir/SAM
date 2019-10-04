#include "general_formulation.h"
#include "utils/check_ptr.h"
#include "utils/log/log.h"
#include "wiringPi.h"
#include <filesystem>
#include <iostream>

GeneralFormulation::GeneralFormulation(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("General Formulation", 0.01)
    , _robot(robot)
    , _Lt(40)
    //    , _lua(30)
    //    , _lfa(20)
    //    , _lwrist(5)
    //    , _lhand(10)
    //    , _lambda(0.)
    , _pin_up(24)
    , _pin_down(22)
    , _lua("lua(cm)", BaseParam::ReadWrite, this, 30)
    , _lfa("lfa(cm)", BaseParam::ReadWrite, this, 35)
    , _lwrist("lwrist(cm)", BaseParam::ReadWrite, this, 10)
    , _lhand("lhand(cm)", BaseParam::ReadWrite, this, 5)
    , _lambdaE("lambda elbow", BaseParam::ReadWrite, this, 0)
    , _lambdaWF("lambda wrist flex", BaseParam::ReadWrite, this, 0)
    , _lambdaWPS("lambda wrist PS", BaseParam::ReadWrite, this, 0)
    , _thresholdE("threshold E", BaseParam::ReadWrite, this, 5)
    , _thresholdWF("threshold WF", BaseParam::ReadWrite, this, 5)
    , _thresholdWPS("threshold WPS", BaseParam::ReadWrite, this, 5)
{
    if (!check_ptr(_robot->joints.elbow_flexion, _robot->joints.wrist_pronation, _robot->sensors.optitrack)) {
        throw std::runtime_error("General Formulation Control is missing components");
    }

    if (!_receiver.bind("0.0.0.0", 45457)) {
        critical() << "General Formulation: Failed to bind receiver for parameters definition";
    }

    _menu->set_description("GeneralFormulation");
    _menu->set_code("gf");
    _menu->add_item("tare", "Tare IMUs", [this](std::string) { this->tare_IMU(); });
    _menu->add_item("calib", "Calibration", [this](std::string) { this->calibrations(); });

    _menu->add_item(_robot->joints.elbow_flexion->menu());
    _menu->add_item(_robot->joints.wrist_pronation->menu());
    _menu->add_item(_robot->joints.hand->menu());

    pullUpDnControl(_pin_up, PUD_UP);
    pullUpDnControl(_pin_down, PUD_UP);

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

GeneralFormulation::~GeneralFormulation()
{
    _robot->joints.elbow_flexion->forward(0);
    _robot->joints.wrist_pronation->forward(0);
    stop_and_join();
}

void GeneralFormulation::tare_IMU()
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

void GeneralFormulation::calibrations()
{
    // HAND
    _robot->joints.hand->take_ownership();
    _robot->joints.hand->init_sequence();
    _robot->joints.hand->move(14);
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

    _robot->joints.elbow_flexion->move_to(-60, 20);
}

void GeneralFormulation::receiveData()
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

void GeneralFormulation::displayPin()
{
    int pin_down_value = digitalRead(_pin_down);
    int pin_up_value = digitalRead(_pin_up);
    debug() << "PinUp: " << pin_up_value;
    debug() << "PinDown: " << pin_down_value;
}

bool GeneralFormulation::setup()
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

    // OPEN AND NAME DATA FILE
    std::string filename("GalF");
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
    _need_to_write_header = true;
    _start_time = clock::now();

    // INITIALIZATION
    _cnt = 0;
    for (int i = 0; i < nbLinks; i++) {
        theta[i] = 0.;
    }
    return true;
}

void GeneralFormulation::loop(double, clock::time_point time)
{
    //    debug() << "Begin loop: " << _cnt << "\n";
    int init_cnt = 10;
    double timeWithDelta = (time - _start_time).count();

    //    receiveData();
    //    if (_robot->joints.wrist_flexion) {
    //        l[0] = _lhand; // from center of the hand to the pronosupination joint
    //        l[1] = _lwrist; //  from the pronosupination joint to the wrist flexion/ext joint
    //        l[2] = _lfa; //  from the wrist flex/ext joint to the elbow flex/ext joint
    //        l[3] = _lua; // from the elbow joint to the acromion/shoulder joint

    //    } else {
    //        l[0] = _lhand + _lwrist;
    //        l[1] = _lfa;
    //        l[2] = _lua;
    //    }

    _robot->sensors.optitrack->update();
    optitrack_data_t data = _robot->sensors.optitrack->get_last_data();
    if (_cnt == 0) {
        debug() << "nbRigid Bodies: " << data.nRigidBodies << "\n";
    }

    double debugData[40];

    /// WRITE FILE HEADERS
    if (_need_to_write_header) {
        _file << " time, pinUp, pinDown,";
        _file << " qWhite.w, qWhite.x, qWhite.y, qWhite.z, qTronc.w, qTronc.x, qTronc.y, qTronc.z,";
        _file << " qFA.w, qFA.x, qFA.y, qFA.z,";
        _file << " to complete";
        _file << " nbRigidBodies";
        for (int i = 0; i < data.nRigidBodies; i++) {
            _file << ", ID, bTrackingValid, fError, qw, qx, qy, qz, x, y, z";
        }
        _file << "\r\n";
        _need_to_write_header = false;
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
    Eigen::Quaterniond qHand;
    qHand.w() = 0.;
    qHand.x() = 0.;
    qHand.y() = 0.;
    qHand.z() = 0.;

    for (int i = 0; i < data.nRigidBodies; i++) {
        if (data.rigidBodies[i].ID == 3) {
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
        } else if (data.rigidBodies[i].ID == 4) {
            //            qHand.w() = data.rigidBodies[i].qw;
            //            qHand.x() = data.rigidBodies[i].qx;
            //            qHand.y() = data.rigidBodies[i].qy;
            //            qHand.z() = data.rigidBodies[i].qz;
            index_hand = i;
        }
    }
    //    debug() << "posA: " << posA[0] << " " << posA[1] << " " << posA[2];

    /// ELBOW
    double elbowEncoder = _robot->joints.elbow_flexion->read_encoder_position();
    /// WRIST
    double pronoSupEncoder = _robot->joints.wrist_pronation->read_encoder_position();
    //    debug() << "pronosup encoder: " << pronoSupEncoder;
    double wristFlexEncoder = 0.;

    /// PROTO with wrist flexor
    if (_robot->joints.wrist_flexion) {
        // set thresholds
        _threshold[0] = _thresholdWF;
        _threshold[1] = _thresholdWPS;
        _threshold[2] = _thresholdE;
        // set gain
        _lambda[0] = _lambdaWF;
        _lambda[1] = _lambdaWPS;
        _lambda[2] = _lambdaE;
        // set arm lengths
        l[0] = _lhand; // from center of the hand to the pronosupination joint
        l[1] = _lwrist; //  from the pronosupination joint to the wrist flexion/ext joint
        l[2] = _lfa; //  from the wrist flex/ext joint to the elbow flex/ext joint
        l[3] = _lua; // from the elbow joint to the acromion/shoulder joint

        wristFlexEncoder = _robot->joints.wrist_flexion->read_encoder_position();
        theta[0] = -wristFlexEncoder / _robot->joints.wrist_flexion->r_incs_per_deg();
        theta[1] = pronoSupEncoder / _robot->joints.wrist_pronation->r_incs_per_deg();
        theta[2] = -elbowEncoder / _robot->joints.elbow_flexion->r_incs_per_deg();
        if (_cnt % 50 == 0)
            debug() << "theta(deg): " << theta[0] << ", " << theta[1] << ", " << theta[2] << "\r\n";
        theta[0] = -M_PI / 2 + theta[0] * M_PI / 180;
        theta[1] = M_PI / 2 + theta[1] * M_PI / 180;
        theta[2] = M_PI / 2 + theta[2] * M_PI / 180;

        debug() << "threshold WPS: " << _threshold[0] << "\n";
        debug() << "threshold WFE: " << _threshold[1] << "\n";
        debug() << "threshold elbow: " << _threshold[2] << "\n";
    }
    /// PROTO without wrist flexor
    else {
        // set thresholds
        _threshold[0] = _thresholdWPS;
        _threshold[1] = _thresholdE;

        // set gain
        _lambda[0] = _lambdaWPS;
        _lambda[1] = _lambdaE;

        // set arm lengths
        l[0] = _lhand + _lwrist;
        l[1] = _lfa;
        l[2] = _lua;

        theta[0] = -pronoSupEncoder / _robot->joints.wrist_pronation->r_incs_per_deg();
        theta[1] = elbowEncoder / _robot->joints.elbow_flexion->r_incs_per_deg();
        if (_cnt % 50 == 0)
            debug() << "theta(deg): " << theta[0] << ", " << theta[1] << "\r\n";
        // in radians:
        theta[0] = theta[0] * M_PI / 180;
        theta[1] = theta[1] * M_PI / 180;

        debug() << "threshold WFE: " << _threshold[0] << "\n";
        debug() << "threshold elbow: " << _threshold[1] << "\n";
    }

    /// IMU
    double qWhite[4], qRed[4], qFA[4];
    if (_robot->sensors.white_imu) {
        _robot->sensors.white_imu->get_quat(qWhite);
        qHand.w() = qWhite[0];
        qHand.x() = qWhite[1];
        qHand.y() = qWhite[2];
        qHand.z() = qWhite[3];
    }
    if (_robot->sensors.red_imu) {
        _robot->sensors.red_imu->get_quat(qRed);
        qHip.w() = qRed[0];
        qHip.x() = qRed[1];
        qHip.y() = qRed[2];
        qHip.z() = qRed[3];
    }
    if (_robot->sensors.yellow_imu)
        _robot->sensors.yellow_imu->get_quat(qFA);

    /// PIN PUSH-BUTTONS CONTROL
    int pin_down_value = digitalRead(_pin_down);
    int pin_up_value = digitalRead(_pin_up);

    /// CONTROL LOOP
    if (_cnt == 0) {
        _lawJ.initialization(posA, qHip, 1 / period());
    } else if (_cnt <= init_cnt) {
        _lawJ.initialPositions(posA, posHip, qHip, _cnt, init_cnt);
        _lawJ.rotationMatrices(qHand, qHip, _cnt, init_cnt);
        _lawJ.projectionInHip(posA, posHip, _cnt, init_cnt);
        _lawJ.updateFrames(theta);
        _lawJ.computeOriginsVectors(l, nbDOF);
    } else {
        _lawJ.rotationMatrices(qHand, qHip, _cnt, init_cnt);
        _lawJ.projectionInHip(posA, posHip, _cnt, init_cnt);
        _lawJ.updateFrames(theta);
        _lawJ.computeOriginsVectors(l, nbDOF);
        _lawJ.controlLaw(posA, _lambda, _threshold, _cnt);

        Eigen::Matrix<double, nbLinks, 1, Eigen::DontAlign> thetaDot_toSend = _lawJ.returnthetaDot_deg();

        if (_robot->joints.wrist_flexion) {
            _robot->joints.wrist_flexion->set_velocity_safe(-thetaDot_toSend[0]);
            _robot->joints.wrist_pronation->set_velocity_safe(thetaDot_toSend[1]);
            _robot->joints.elbow_flexion->set_velocity_safe(-thetaDot_toSend[2]);
            if (_cnt % 50 == 0) {
                debug() << "wrist flex vel :" << thetaDot_toSend[0] << "\n";
                debug() << "pronosup vel :" << thetaDot_toSend[1] << "\n";
                debug() << "elbow flex vel :" << thetaDot_toSend[2] << "\n";
            }
        } else {
            _robot->joints.wrist_pronation->set_velocity_safe(-thetaDot_toSend[0]);
            _robot->joints.elbow_flexion->set_velocity_safe(thetaDot_toSend[1]);
            if (_cnt % 50 == 0) {
                debug() << "pronosup vel :" << thetaDot_toSend[0] << "\n";
                debug() << "elbow flex vel :" << thetaDot_toSend[1] << "\n";
            }
        }
    }

    _lawJ.writeDebugData(debugData, theta);
    /// WRITE DATA
    _file << nbDOF << timeWithDelta << ' ' << pin_down_value << ' ' << pin_up_value << ' ' << _lua << ' ' << _lfa << ' ' << _lwrist << ' ' << _lhand;
    _file << ' ' << _lambda << ' ' << _threshold[0] << ' ' << _threshold[1] << ' ' << _threshold[2];
    _file << ' ' << qWhite[0] << ' ' << qWhite[1] << ' ' << qWhite[2] << ' ' << qWhite[3] << ' ' << qRed[0] << ' ' << qRed[1] << ' ' << qRed[2] << ' ' << qRed[3];
    //    _file << ' ' << qFA[0] << ' ' << qFA[1] << ' ' << qFA[2] << ' ' << qFA[3];
    for (int i = 0; i < 40; i++) {
        _file << ' ' << debugData[i];
    }
    _file << ' ' << pronoSupEncoder << ' ' << wristFlexEncoder << ' ' << elbowEncoder;
    _file << ' ' << data.nRigidBodies;

    for (int i = 0; i < data.nRigidBodies; i++) {
        _file << ' ' << data.rigidBodies[i].ID << ' ' << data.rigidBodies[i].bTrackingValid << ' ' << data.rigidBodies[i].fError;
        _file << ' ' << data.rigidBodies[i].qw << ' ' << data.rigidBodies[i].qx << ' ' << data.rigidBodies[i].qy << ' ' << data.rigidBodies[i].qz;
        _file << ' ' << data.rigidBodies[i].x << ' ' << data.rigidBodies[i].y << ' ' << data.rigidBodies[i].z;
    }
    _file << std::endl;

    ++_cnt;
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()).count() << "ms" << std::endl;
}

void GeneralFormulation::cleanup()
{
    _robot->joints.wrist_pronation->forward(0);
    if (_robot->joints.wrist_flexion)
        _robot->joints.wrist_flexion->forward(0);
    //    _robot->joints.elbow_flexion->move_to(0, 20);
    _robot->joints.hand->release_ownership();
    _file.close();
}
