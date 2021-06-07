#include "matlab_optimization.h"
#include "components/internal/hand/touch_bionics_hand.h"
#include "utils/check_ptr.h"
#include "utils/log/log.h"
#include <filesystem>
#include <iostream>
#include <sstream>

MatlabOptimization::MatlabOptimization(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("Matlab Optimization", 0.01)
    , _robot(robot)
{
    if (!check_ptr(_robot->joints.elbow_flexion, _robot->joints.wrist_pronation, _robot->sensors.optitrack)) {
        throw std::runtime_error("Matlab Optimization is missing components");
    }

    _menu->set_description("Matlab Optimization");
    _menu->set_code("mo");

    int port = 45455;
    if (!_socket.bind("0.0.0.0", 45455)) {
        critical() << "Failed to bind on port" << port;
    }

    _menu->add_item("calib", "Calibration", [this](std::string) { this->calibrations(); });
    _menu->add_item("elbow90", "Flex elbow at 90 deg", [this](std::string) { this->elbowTo90(); });
    _menu->add_item(_robot->joints.wrist_pronation->menu());
    _menu->add_item(_robot->joints.elbow_flexion->menu());
    if (_robot->joints.hand)
        _menu->add_item(_robot->joints.hand->menu());
}

MatlabOptimization::~MatlabOptimization()
{
    _robot->joints.elbow_flexion->forward(0);
    _robot->joints.wrist_pronation->forward(0);
    _socket.close();
    stop_and_join();
}

void MatlabOptimization::elbowTo90()
{
    if (protoCyb)
        _robot->joints.elbow_flexion->move_to(90, 20);
    else
        _robot->joints.elbow_flexion->move_to(-90, 10);
}

void MatlabOptimization::calibrations()
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

    //    if (protoCyb)
    //        _robot->joints.elbow_flexion->move_to(100, 20, true);
    //    else
    //        _robot->joints.elbow_flexion->move_to(-90, 20);
}

void MatlabOptimization::set_velocity_motors(double speed1, double speed2)
{
    // ELBOW
    if (_robot->joints.elbow_flexion->is_calibrated() == false) {
        warning() << "Elbow not calibrated...";
        return;
    }
    int32_t pos_elbow;
    double max_angle_elbow = _robot->joints.elbow_flexion->get_max_angle();
    double min_angle_elbow = _robot->joints.elbow_flexion->get_min_angle();
    if (speed1 > 0)
        pos_elbow = static_cast<int32_t>(max_angle_elbow * _robot->joints.elbow_flexion->r_incs_per_deg());
    else
        pos_elbow = static_cast<int32_t>(min_angle_elbow * _robot->joints.elbow_flexion->r_incs_per_deg());

    uint32_t velocity_elbow = static_cast<uint32_t>(std::fabs(speed1) * _robot->joints.elbow_flexion->r_incs_per_deg());
    if (velocity_elbow == 0)
        velocity_elbow = 1;
    uint32_t acc_elbow = _robot->joints.elbow_flexion->get_acc();

    // WRIST PRONATION
    int32_t pos_wrist;
    int32_t min_angle_wrist = -130;
    int32_t max_angle_wrist = 90;
    if (speed2 > 0)
        pos_wrist = max_angle_wrist * static_cast<int32_t>(_robot->joints.wrist_pronation->r_incs_per_deg());
    else
        pos_wrist = min_angle_wrist * static_cast<int32_t>(_robot->joints.wrist_pronation->r_incs_per_deg());

    uint32_t velocity_wrist = static_cast<uint32_t>(std::fabs(speed2) * _robot->joints.wrist_pronation->r_incs_per_deg());
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

bool MatlabOptimization::setup()
{
    _robot->joints.wrist_pronation->set_encoder_position(0);

    _lambda[0] = 2;
    _lambda[1] = 2;
    _threshold[0] = 5;
    _threshold[1] = 5;
    for (int i = 0; i < sizeof(_qd) / sizeof(*_qd); i++) {
        _qd[i] = 400.;
    }
    // OPEN AND NAME DATA FILE
    if (saveData) {
        std::string filename("MatlabOpt");
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
    for (int i = 0; i < 2; i++) {
        _theta[i] = 0.;
        _thetaDiff[i] = 0.;
    }
    return true;
}

void MatlabOptimization::loop(double, clock::time_point time)
{
    //    _robot->sensors.optitrack->update();
    //    optitrack_data_t data = _robot->sensors.optitrack->get_last_data();

    static double pronoSupEncoder = 0;
    static double elbowEncoder = 0;
    static double tmpEncoder = 0;

    int init_cnt = 10;
    double timeWithDelta = (time - _start_time).count();
    //    std::cout << timeWithDelta << std::endl;

    /// READ ENCODER WRIST
    try {
        tmpEncoder = pronoSupEncoder;
        pronoSupEncoder = _robot->joints.wrist_pronation->read_encoder_position();
    } catch (std::runtime_error& e) {
        std::cout << "Runtime error when reading wrist encoder: " << e.what() << std::endl;
        pronoSupEncoder = tmpEncoder;
    }
    if (protoCyb) {
        _theta[0] = pronoSupEncoder / _robot->joints.wrist_pronation->r_incs_per_deg();
        _theta[1] = elbowEncoder / _robot->joints.elbow_flexion->r_incs_per_deg();
    } else {
        _theta[0] = pronoSupEncoder / _robot->joints.wrist_pronation->r_incs_per_deg();
        _theta[1] = elbowEncoder / _robot->joints.elbow_flexion->r_incs_per_deg();
    }
    listenMatlab();
    //    std::cout << "Qd elbow and wrist pronosup (rad): " << _qd[0] - M_PI << ";" << _qd[1] << std::endl;
    //    std::cout << "Theta elbow and wrist pronosup (rad): " << _theta[1] << "; " << _theta[0] << std::endl;
    /// READ ENCODER ELBOW
    try {
        tmpEncoder = elbowEncoder;
        elbowEncoder = _robot->joints.elbow_flexion->read_encoder_position();
    } catch (std::runtime_error& e) {
        std::cout << "Runtime error when reading elbow encoder: " << e.what() << std::endl;
        elbowEncoder = tmpEncoder;
    }

    /// SEND COMMAND TO JOINT MOTORS

    if ((_qd[0] < 400.) && (_qd[1] < 400.)) {

        if (protoCyb) {
            _thetaDiff[0] = _qd[1] * 180 / M_PI;
            _thetaDiff[1] = -_qd[0] * 180 / M_PI;
        } else {
            _thetaDiff[0] = _qd[1] * 180 / M_PI;
            _thetaDiff[1] = _qd[0] * 180 / M_PI;
        }

        std::cout << "Wrist diff (deg): " << _thetaDiff[0] << "; Elbow diff (deg): " << _thetaDiff[1] << std::endl;
        std::cout << "" << std::endl;

        for (int i = 0; i < 2; i++) {
            //            // Speed proportional to difference between current and optimal angular posture
            //            if (abs(_thetaDiff[i]) < _threshold[i])
            //                _thetaDot[i] = 0.;
            //            else
            //                _thetaDot[i] = _lambda[i] * _thetaDiff[i];

            //            // Constant speed
            //            if (abs(_thetaDiff[i]) < _threshold[i])
            //                _thetaDot[i] = 0.;
            //            else {
            //                _thetaDot[0] = _thetaDiff[0] / abs(_thetaDiff[0]) * 40;
            //                _thetaDot[1] = _thetaDiff[1] / abs(_thetaDiff[1]) * 20;
            //            }

            // Position control
            if (abs(_thetaDiff[i]) > _threshold[i]) {
                // virtual stops
                if ((_theta[0] + _thetaDiff[0]) > 180)
                    _robot->joints.wrist_pronation->move_to(180, 60);
                else if ((_theta[0] + _thetaDiff[0]) < -180)
                    _robot->joints.wrist_pronation->move_to(-180, 60);
                else if ((_theta[1] + _thetaDiff[1]) > 100)
                    _robot->joints.elbow_flexion->move_to(100, 10);
                else if ((_theta[1] + _thetaDiff[1]) < 0)
                    _robot->joints.elbow_flexion->move_to(2, 10);
                else {
                    _robot->joints.wrist_pronation->move_to(_theta[0] + _thetaDiff[0], 60);
                    _robot->joints.elbow_flexion->move_to(_theta[1] + _thetaDiff[1], 10);
                }
                _thetaDiff[0] = 0.;
                _thetaDiff[1] = 0.;
                _qd[0] = 400;
                _qd[1] = 400;
            }
        }

        //        std::cout << "Send velocity commands (deg): " << _thetaDot[0] << "; " << _thetaDot[1] << std::endl;
        //        set_velocity_motors(_thetaDot[1], _thetaDot[0]);
        //        _robot->joints.elbow_flexion->set_velocity_safe(-_thetaDot[1]);
    }
    if (saveData) {
        /// WRITE DATA
        _file << ' ' << timeWithDelta;
        // _file << ' ' << _lambda[0] << ' ' << _lambda[1];
        _file << ' ' << pronoSupEncoder << ' ' << elbowEncoder << ' ' << _theta[0] << ' ' << _theta[1];
        for (int i = 0; i < 2; i++) {
            _file << ' ' << _qd[i];
        }
        // _file << ' ' << data.nRigidBodies;

        //        for (int i = 0; i < nbRigidBodies; i++) {
        //            _file << ' ' << data.rigidBodies[i].ID << ' ' << data.rigidBodies[i].bTrackingValid << ' ' << data.rigidBodies[i].fError;
        //            _file << ' ' << data.rigidBodies[i].qw << ' ' << data.rigidBodies[i].qx << ' ' << data.rigidBodies[i].qy << ' ' << data.rigidBodies[i].qz;
        //            _file << ' ' << data.rigidBodies[i].x << ' ' << data.rigidBodies[i].y << ' ' << data.rigidBodies[i].z;
        //        }
        _file << std::endl;
    }
    ++_cnt;
}

void MatlabOptimization::listenMatlab()
{
    /// READ DATA FROM MATLAB
    if (_socket.available()) {
        std::cout << "Matlab socket available " << _cnt << std::endl;
        auto data = _socket.receive();
        std::string buf;
        buf.resize(data.size());
        std::transform(data.begin(), data.end(), buf.begin(), [](std::byte b) -> char { return static_cast<char>(b); });

        std::cout << buf << std::endl;
        std::istringstream ts(buf);
        for (int i = 0; i < (sizeof(_qd) / sizeof(*_qd)); i++) {
            ts >> _qd[i];
        }
    }
}
void MatlabOptimization::cleanup()
{
    _robot->joints.wrist_pronation->forward(0);
    _robot->joints.elbow_flexion->set_velocity_safe(0);
    _file.close();
}
