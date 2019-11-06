#include "compensation_optitrack.h"
#include "utils/check_ptr.h"
#include "utils/log/log.h"
#include <filesystem>

CompensationOptitrack::CompensationOptitrack(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("compensation_optitrack", 0.01)
    , _robot(robot)
    , _Lt(40)
    , _Lua(0.)
    , _Lfa(0.)
    , _l(0.)
    , _lsh(-35)
    , _lambdaW(0)
    , _lambda(0)
    , _thresholdW(5.)
    , _pinArduino(0)
{
    if (!check_ptr(_robot->joints.elbow_flexion, _robot->joints.wrist_pronation, _robot->sensors.trunk_imu, _robot->sensors.arm_imu, _robot->sensors.optitrack)) {
        throw std::runtime_error("Optitrack Compensation is missing components");
    }

    if (!_receiver.bind("0.0.0.0", 45454)) {
        critical() << "CompensationOptitrack: Failed to bind receiver";
    }

    if (!_receiverArduino.bind("0.0.0.0", 45455)) {
        critical() << "CompensationOptitrack: Failed to bind arduino receiver";
    }

    _menu->set_description("Control with optitrack recording");
    _menu->set_code("opti");
    _menu->add_item("1", "Start (+ filename [comp for compensation, vol for voluntary control])", [this](std::string args) { this->start(args); });
    _menu->add_item("zero", "Back to 0°", [this](std::string) { this->zero(); });
    _menu->add_item("disp", "Display law parameters", [this](std::string) { this->display_parameters(); });
    _menu->add_item("al", "Display anatomical lengths", [this](std::string) { this->display_lengths(); });

    _menu->add_item(_robot->joints.elbow_flexion->menu());
    _menu->add_item(_robot->joints.wrist_pronation->menu());
    _menu->add_item(_robot->joints.hand->menu());

    _menu->set_activated_callback([this] { on_activated(); });

    _abs_time_start = clock::now();
}

CompensationOptitrack::~CompensationOptitrack()
{
    stop_and_join();
}

void CompensationOptitrack::zero()
{
    _robot->joints.elbow_flexion->move_to(0, 10, true);
}

void CompensationOptitrack::tareIMU()
{
    _robot->sensors.arm_imu->send_command_algorithm_init_then_tare();
    _robot->sensors.trunk_imu->send_command_algorithm_init_then_tare();
    debug("Wait for triple bip");

    std::this_thread::sleep_for(std::chrono::seconds(6));
    _robot->user_feedback.buzzer->makeNoise(Buzzer::TRIPLE_BUZZ);

    double qBras[4], qTronc[4];
    _robot->sensors.arm_imu->get_quat(qBras);
    _robot->sensors.trunk_imu->get_quat(qTronc);
    debug() << "IMU Bras : " << qBras[0] << " " << qBras[1] << " " << qBras[2] << " " << qBras[3];
    debug() << "IMU Tronc : " << qTronc[0] << " " << qTronc[1] << " " << qTronc[2] << " " << qTronc[3];
}

void CompensationOptitrack::display_parameters()
{
    debug() << "lambda: " << _lambda;
    debug() << "Lfa: " << _Lfa;
    debug() << "Lua: " << _Lua;
    debug() << "l: " << _l;
    debug() << "Threshold (in rad): " << _threshold;
    debug() << "Threshold wrist (in rad): " << _thresholdW;
    debug() << "lambda wrist: " << _lambdaW;
}

void CompensationOptitrack::display_lengths()
{
    _ind = 0;
    info() << "Wait for Optitrack data";
    printf("Wait for Optitrack data");
    start();
}

void CompensationOptitrack::read_optiData(optitrack_data_t data)
{
    unsigned int opti_freq = 100;

    int index_acromion = -1, index_EE = -1, index_elbow = -1, index_hip = -1;
    Eigen::Vector3f posA = Eigen::Vector3f::Zero();
    Eigen::Vector3f posElbow = Eigen::Vector3f::Zero();
    Eigen::Vector3f posFA = Eigen::Vector3f::Zero();
    Eigen::Vector3f posEE = Eigen::Vector3f::Zero();
    Eigen::Vector3f posHip = Eigen::Vector3f::Zero();
    Eigen::Quaternionf qHip, qFA_record;

    qHip.w() = 0.;
    qFA_record.w() = 0.;
    qHip.x() = 0.;
    qFA_record.x() = 0.;
    qHip.y() = 0.;
    qFA_record.y() = 0.;
    qHip.z() = 0.;
    qFA_record.z() = 0.;

    for (unsigned int i = 0; i < data.nRigidBodies; i++) {
        if (data.rigidBodies[i].ID == 3) {
            posA[0] = data.rigidBodies[i].x * 100;
            posA[1] = data.rigidBodies[i].y * 100;
            posA[2] = data.rigidBodies[i].z * 100;
            index_acromion = static_cast<int>(i);
        } else if (data.rigidBodies[i].ID == 4) {
            posFA[0] = data.rigidBodies[i].x * 100;
            posFA[1] = data.rigidBodies[i].y * 100;
            posFA[2] = data.rigidBodies[i].z * 100;

            qFA_record.w() = data.rigidBodies[i].qw;
            qFA_record.x() = data.rigidBodies[i].qx;
            qFA_record.y() = data.rigidBodies[i].qy;
            qFA_record.z() = data.rigidBodies[i].qz;
        } else if (data.rigidBodies[i].ID == 6) {
            posElbow[0] = data.rigidBodies[i].x * 100;
            posElbow[1] = data.rigidBodies[i].y * 100;
            posElbow[2] = data.rigidBodies[i].z * 100;
            index_elbow = static_cast<int>(i);
        } else if (data.rigidBodies[i].ID == 9) {
            posEE[0] = data.rigidBodies[i].x * 100;
            posEE[1] = data.rigidBodies[i].y * 100;
            posEE[2] = data.rigidBodies[i].z * 100;
            index_EE = static_cast<int>(i);
        } else if (data.rigidBodies[i].ID == 10) {
            posHip[0] = data.rigidBodies[i].x * 100;
            posHip[1] = data.rigidBodies[i].y * 100;
            posHip[2] = data.rigidBodies[i].z * 100;
            qHip.w() = data.rigidBodies[i].qw;
            qHip.x() = data.rigidBodies[i].qx;
            qHip.y() = data.rigidBodies[i].qy;
            qHip.z() = data.rigidBodies[i].z;
            index_hip = static_cast<int>(i);
        }
    }

    _lawopti.initialization(posA, posEE, posHip, qHip, opti_freq);
    _lawopti.rotationMatrices(qHip, qFA_record, 1, 10);
    _lawopti.computeEEfromFA(posFA, _l, qFA_record);
    _lawopti.projectionInHip(posA, posElbow, posHip, 1, 10);
    _Lua = std::round((_lawopti.returnPosElbowinHip() - _lawopti.returnPosAinHip()).norm());
    _Lfa = std::round((posElbow - posFA).norm());
    _l = std::round((posFA - posEE).norm());

    if (_ind == 0) {
        debug() << "posA: " << posA[0] << " " << posA[1] << " " << posA[2];
        debug() << "posFA: " << posFA[0] << " " << posFA[1] << " " << posFA[2];
        debug() << "posEE: " << posEE[0] << " " << posEE[1] << " " << posEE[2];
        debug() << "posHip: " << posHip[0] << " " << posHip[1] << " " << posHip[2];
        debug() << "qHip: " << qHip.w() << ", " << qHip.x() << ", " << qHip.y() << ", " << qHip.z();
        debug() << "Lua: " << _Lua << ", Lfa: " << _Lfa << ", l: " << _l;
        _ind = 1;
    }
}

void CompensationOptitrack::start(std::string filename)
{
    if (filename.empty())
        filename = "test";

    int cnt = 0;
    std::string suffix;
    std::string extension = std::string(".txt");
    do {
        ++cnt;
        suffix = std::string("_") + std::to_string(cnt);
    } while (std::filesystem::exists(filename + suffix + extension));

    _file = std::ofstream(filename + suffix + extension);

    if (!_file.good()) {
        critical() << "Failed to open " << (filename + suffix + extension);
        return;
    }
    _need_to_write_header = true;

    _cnt = 0;
    _infoSent = 0;

    if (filename == std::string("comp")) {
        _mode = COMP;
    } else if (filename == std::string("vol")) {
        _mode = VOL;
    } else {
        debug() << "Filename does not correspond to one of the control mode.\n 'opti' for compensation control; 'vol', for voluntary";
    }

    _time_start = clock::now();

    ThreadedLoop::start();
}

void CompensationOptitrack::stop()
{
    _infoSent = 0;
    _robot->joints.elbow_flexion->forward(0);
    _robot->joints.wrist_pronation->forward(0);

    _file.close();
}

bool CompensationOptitrack::setup()
{
    on_def();
    return true;
}

void CompensationOptitrack::loop(double dt, clock::time_point time)
{
    listenArduino();
    _robot->sensors.optitrack->update();

    if (_mode == COMP) {
        on_new_data_compensation(_robot->sensors.optitrack->get_last_data(), dt, time);
    } else if (_mode == VOL) {
        on_new_data_vol(_robot->sensors.optitrack->get_last_data(), dt, time);
    }
}

void CompensationOptitrack::cleanup()
{
}

void CompensationOptitrack::on_new_data_compensation(optitrack_data_t data, double dt, clock::time_point time)
{
    const unsigned int init_cnt = 10;
    int btn_sync = 0;

    Eigen::Vector3f posA, posElbow, posFA, posEE, posHip;
    Eigen::Quaternionf qHip, qFA_record;
    double timeWithDelta = (time - _time_start).count();
    double deltaTtable = dt;
    double absTtable = (time - _abs_time_start).count();

    int timerTask = 1;

    double qBras[4], qTronc[4];
    _robot->sensors.arm_imu->get_quat(qBras);
    _robot->sensors.trunk_imu->get_quat(qTronc);
    //    qDebug() << "IMU Bras : " << qBras[0] << " " << qBras[1] << " " << qBras[2] << " " << qBras[3];
    //    qDebug() << "IMU Tronc : " << qTronc[0] << " " << qTronc[1] << " " << qTronc[2] << " " << qTronc[3];

    double debugData[35];

    int index_acromion = -1, index_FA = -1, index_EE = -1, index_elbow = -1, index_hip = -1;

    for (unsigned int i = 0; i < data.nRigidBodies; i++) {
        if (data.rigidBodies[i].ID == 3) {
            posA[0] = data.rigidBodies[i].x * 100;
            posA[1] = data.rigidBodies[i].y * 100;
            posA[2] = data.rigidBodies[i].z * 100;
            index_acromion = static_cast<int>(i);
        } else if (data.rigidBodies[i].ID == 4) {
            posFA[0] = data.rigidBodies[i].x * 100;
            posFA[1] = data.rigidBodies[i].y * 100;
            posFA[2] = data.rigidBodies[i].z * 100;
            qFA_record.w() = data.rigidBodies[i].qw;
            qFA_record.x() = data.rigidBodies[i].qx;
            qFA_record.y() = data.rigidBodies[i].qy;
            qFA_record.z() = data.rigidBodies[i].qz;
            index_FA = static_cast<int>(i);
        } else if (data.rigidBodies[i].ID == 6) {
            posElbow[0] = data.rigidBodies[i].x * 100;
            posElbow[1] = data.rigidBodies[i].y * 100;
            posElbow[2] = data.rigidBodies[i].z * 100;
            index_elbow = static_cast<int>(i);
        } else if (data.rigidBodies[i].ID == 9) {
            posEE[0] = data.rigidBodies[i].x * 100;
            posEE[1] = data.rigidBodies[i].y * 100;
            posEE[2] = data.rigidBodies[i].z * 100;
            index_EE = static_cast<int>(i);
        } else if (data.rigidBodies[i].ID == 10) {
            posHip[0] = data.rigidBodies[i].x * 100;
            posHip[1] = data.rigidBodies[i].y * 100;
            posHip[2] = data.rigidBodies[i].z * 100;
            qHip.w() = data.rigidBodies[i].qw;
            qHip.x() = data.rigidBodies[i].qx;
            qHip.y() = data.rigidBodies[i].qy;
            qHip.z() = data.rigidBodies[i].qz;
            index_hip = static_cast<int>(i);
        }
    }

    double beta = _robot->joints.elbow_flexion->pos() * M_PI / 180.;

    if (_need_to_write_header) {
        _file << "delta, time, btn_sync, abs_time, emg1, emg2, timerTask, pinArduino,";
        _file << " qBras.w, qBras.x, qBras.y, qBras.z, qTronc.w, qTronc.x, qTronc.y, qTronc.z";
        _file << " index_acromion, index_EE, index_elbow, initialAcromionPosition.x, initialAcromionPosition.y, initialAcromionPosition.z, AcromionPosition.x, AcromionPosition.y, AcromionPosition.z,";
        _file << " positionEE_inHip.x, positionEE_inHip.y, positionEE_inHip.z, delta, beta_new, beta, dBeta, betaDot, lambda, threshold, nbRigidBodies,";
        _file << " phi wrist, theta wrist, wrist angle, wristAngVel, lambdaW, thresholdW";
        for (unsigned int i = 0; i < data.nRigidBodies; i++) {
            _file << ", ID, bTrackingValid, fError, qw, qx, qy, qz, x, y, z";
        }
        _file << "\r\n";
        _need_to_write_header = false;
    }

    if (_cnt == 0) {
        const unsigned int opti_freq = 100;
        if (_Lua == 0 && _Lfa == 0) {
            //            _Lua = qRound((posElbow - posA).norm());
            //            //            _Lfa = qRound((posElbow - posEE).norm());
            //            _Lfa = qRound((posElbow - posFA).norm());
            //            _l = qRound((posFA - posEE).norm());
            //            qDebug("Lua: %lf, Lfa: %lf, l: %lf", _Lua, _Lfa, _l);
            debug("Anatomical lengths not defined");
        }
        _lawopti.initialization(posA, posEE, posHip, qHip, opti_freq);
    } else if (_cnt <= init_cnt) {
        _lawopti.initialPositions(posA, posHip, qHip, qFA_record, _cnt, init_cnt);
        if (_cnt == init_cnt) {
            /// Computation posA0 in hip frame and move posA0 in prosthetic arm plane
            _lawopti.rotationMatrices(qHip, qFA_record, _cnt, init_cnt);
            //            _lawopti.computeEEfromFA(posFA, _l, qFA_record);
            //            _lawopti.projectionInHip(posA, posElbow, posHip, _cnt, init_cnt);
            _lawopti.bufferingOldValues();
        }
        _lawopti.filter_optitrackData(posA, posEE);
    } else {
        _lawopti.rotationMatrices(qHip, qFA_record, _cnt, init_cnt);
        //        _lawopti.computeEEfromFA(posFA, _l, qFA_record);
        //        _lawopti.projectionInHip(posA, posElbow, posHip, _cnt, init_cnt);
        //        _lawopti.controlLaw(posEE, beta, _Lua, _Lfa, _l, _lambda, _threshold);
        _lawopti.controlLawWrist(_lambdaW, _thresholdW);
        _robot->joints.elbow_flexion->set_velocity_safe(_lawopti.returnBetaDot_deg());

        if (_lawopti.returnWristVel_deg() > 0)
            _robot->joints.wrist_pronation->move_to(6000, _lawopti.returnWristVel_deg() * 100, 6000, 35000);
        else if (_lawopti.returnWristVel_deg() < 0)
            _robot->joints.wrist_pronation->move_to(6000, -_lawopti.returnWristVel_deg() * 100, 6000, -35000);
        else if (_lawopti.returnWristVel_deg() == 0)
            _robot->joints.wrist_pronation->forward(0);

        _lawopti.bufferingOldValues();

        if (_cnt % 50 == 0) {
            // _lawopti.displayData(posEE, beta);
            //            qDebug() << "betaDot in deg:" << _lawopti.returnBetaDot_deg();
        }
        // buzzer after 1s, to indicate the start of the task
        if (_cnt == 100) {
            _robot->user_feedback.buzzer->makeNoise(Buzzer::STANDARD_BUZZ);
        }
    }

    _lawopti.writeDebugData(debugData, posEE, beta);

    if (_cnt == 0) {
        _file << _Lua << ' ' << _Lfa << ' ' << _l << std::endl;
    }
    _file << deltaTtable << ' ' << timeWithDelta << ' ' << btn_sync << ' ' << absTtable << ' ' << _robot->sensors.adc0->readADC_SingleEnded(0) << ' ' << _robot->sensors.adc0->readADC_SingleEnded(1) << ' ' << timerTask;
    _file << ' ' << _pinArduino;
    _file << ' ' << qBras[0] << ' ' << qBras[1] << ' ' << qBras[2] << ' ' << qBras[3] << ' ' << qTronc[0] << ' ' << qTronc[1] << ' ' << qTronc[2] << ' ' << qTronc[3];
    _file << ' ' << index_acromion << ' ' << index_EE << ' ' << index_elbow << ' ' << debugData[0] << ' ' << debugData[1] << ' ' << debugData[2] << ' ' << posA[0] << ' ' << posA[1] << ' ' << posA[2];
    _file << ' ' << debugData[3] << ' ' << debugData[4] << ' ' << debugData[5] << ' ' << debugData[6] << ' ' << debugData[7] << ' ' << debugData[8];
    _file << ' ' << debugData[9] << ' ' << debugData[10] << ' ' << debugData[11] << ' ' << debugData[12] << ' ' << _lambda << ' ' << _threshold << ' ' << data.nRigidBodies;
    _file << ' ' << debugData[13] << ' ' << debugData[14] << ' ' << debugData[15] << ' ' << debugData[16] << ' ' << _lambdaW << ' ' << _thresholdW;

    for (unsigned int i = 0; i < data.nRigidBodies; i++) {
        _file << ' ' << data.rigidBodies[i].ID << ' ' << data.rigidBodies[i].bTrackingValid << ' ' << data.rigidBodies[i].fError;
        _file << ' ' << data.rigidBodies[i].qw << ' ' << data.rigidBodies[i].qx << ' ' << data.rigidBodies[i].qy << ' ' << data.rigidBodies[i].qz;
        _file << ' ' << data.rigidBodies[i].x << ' ' << data.rigidBodies[i].y << ' ' << data.rigidBodies[i].z;
    }
    _file << std::endl;

    ++_cnt;

    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()).count() << "ms" << std::endl;
}

void CompensationOptitrack::on_new_data_vol(optitrack_data_t data, double dt, clock::time_point time)
{
    double timeWithDelta = (time - _time_start).count();

    // buzzer after 1s, to indicate the start of the task
    if (_cnt == 50) {
        _robot->user_feedback.buzzer->makeNoise(Buzzer::STANDARD_BUZZ);
    }
    _old_time = timeWithDelta;
    static int prev_pin_up_value = 1, prev_pin_down_value = 1;
    int pin_down_value = _robot->btn2;
    int pin_up_value = _robot->btn1;

    /// ELBOW
    //    double beta = _osmer.angle() * M_PI / 180.;

    //    if (pin_down_value == 0 && prev_pin_down_value == 1) {
    //        _osmer.set_velocity(30);
    //    } else if (pin_up_value == 0 && prev_pin_up_value == 1) {
    //        _osmer.set_velocity(-30);
    //    } else if ((pin_down_value == 1 && pin_up_value == 1) && (prev_pin_down_value == 0 || prev_pin_up_value == 0)) {
    //        _osmer.set_velocity(0);
    //        //        sleep(2);
    //        //        _osmer.set_velocity(0);
    //    }

    /// WRIST
    double wristAngle = _robot->joints.wrist_pronation->read_encoder_position();

    if (pin_down_value == 0 && prev_pin_down_value == 1) {
        _robot->joints.wrist_pronation->move_to(6000, 5000, 6000, 35000);
    } else if (pin_up_value == 0 && prev_pin_up_value == 1) {
        _robot->joints.wrist_pronation->move_to(6000, 5000, 6000, -35000);
    } else if ((pin_down_value == 1 && pin_up_value == 1) && (prev_pin_down_value == 0 || prev_pin_up_value == 0)) {
        _robot->joints.wrist_pronation->forward(0);
    }

    prev_pin_down_value = pin_down_value;
    prev_pin_up_value = pin_up_value;

    double qBras[4], qTronc[4];
    _robot->sensors.arm_imu->get_quat(qBras);
    _robot->sensors.trunk_imu->get_quat(qTronc);
    if (_need_to_write_header) {
        //        _file.write("period, btnUp, btnDown, beta");
        _file << "time, btnUp, btnDown, pinArduino, wristAngle,";
        _file << " qBras.w, qBras.x, qBras.y, qBras.z, qTronc.w, qTronc.x, qTronc.y, qTronc.z";
        _file << " nbRigid Bodies";
        for (unsigned int i = 0; i < data.nRigidBodies; i++) {
            _file << ", ID, bTrackingValid, fError, qw, qx, qy, qz, x, y, z";
        }
        _file << std::endl;
        _need_to_write_header = false;
    }

    //    ts << return_period() << ' ' << pin_up_value << ' ' << pin_down_value << ' ' << beta;
    _file << timeWithDelta << ' ' << pin_up_value << ' ' << pin_down_value << ' ' << _pinArduino << ' ' << wristAngle;
    _file << ' ' << qBras[0] << ' ' << qBras[1] << ' ' << qBras[2] << ' ' << qBras[3] << ' ' << qTronc[0] << ' ' << qTronc[1] << ' ' << qTronc[2] << ' ' << qTronc[3];
    _file << ' ' << data.nRigidBodies;

    for (unsigned int i = 0; i < data.nRigidBodies; i++) {
        _file << ' ' << data.rigidBodies[i].ID << ' ' << data.rigidBodies[i].bTrackingValid << ' ' << data.rigidBodies[i].fError;
        _file << ' ' << data.rigidBodies[i].qw << ' ' << data.rigidBodies[i].qx << ' ' << data.rigidBodies[i].qy << ' ' << data.rigidBodies[i].qz;
        _file << ' ' << data.rigidBodies[i].x << ' ' << data.rigidBodies[i].y << ' ' << data.rigidBodies[i].z;
    }
    _file << std::endl;
}

void CompensationOptitrack::on_activated()
{
    _robot->joints.elbow_flexion->calibrate();
    _robot->joints.wrist_pronation->set_encoder_position(0);
}

void CompensationOptitrack::on_def()
{
    while (_receiver.available()) {
        auto data = _receiver.receive();

        std::string buf;
        buf.resize(data.size());
        std::transform(data.begin(), data.end(), buf.begin(), [](std::byte b) -> char { return static_cast<char>(b); });

        debug() << buf;

        std::istringstream ts(buf);
        int tmp;

        ts >> tmp;
        _Lua = tmp;

        ts >> tmp;
        _Lfa = tmp;

        ts >> tmp;
        _l = tmp;

        ts >> tmp;
        _lambda = tmp;

        ts >> tmp;
        _lambdaW = tmp;

        ts >> tmp;
        _threshold = tmp * M_PI / 180.; // dead zone limit for beta change, in rad.

        ts >> tmp;
        _thresholdW = tmp * M_PI / 180; // dead zone limit for wrist angle change, in rad.
    }
}

void CompensationOptitrack::listenArduino()
{
    while (_receiverArduino.available()) {
        if (_infoSent == 0) {
            debug("Listening Arduino");
            _infoSent = 1;
        }
        auto data = _receiverArduino.receive();
        std::string buf;
        std::transform(data.begin(), data.end(), buf.begin(), [](std::byte b) { return static_cast<char>(b); });
        _pinArduino = std::stoi(buf);
    }
}
