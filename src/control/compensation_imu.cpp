#include "compensation_imu.h"
#include "utils/check_ptr.h"
#include "utils/log/log.h"
#include <filesystem>

CompensationIMU::CompensationIMU(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("Compensation IMU", 0.01)
    , _robot(robot)
    , _Lt(40)
    , _Lua(0.)
    , _Lfa(0.)
    , _l(0.)
    , _lambdaW(0)
    , _lambda(0)
    , _thresholdW(5.)
{
    if (!check_ptr(_robot->joints.elbow_flexion, _robot->joints.wrist_pronation, _robot->sensors.fa_imu)) {
        throw std::runtime_error("Compensation IMU Control is missing components");
    }

    if (!_receiver.bind("0.0.0.0", 45454)) {
        critical() << "CompensationOptitrack: Failed to bind receiver";
    }

    _menu->set_description("CompensationIMU");
    _menu->set_code("imu");
    _menu->add_item("Tare IMUs", "tare", [this](std::string) { this->tare_IMU(); });
    _menu->add_item("Display Pin data", "pin", [this](std::string) { this->displayPin(); });

    _menu->add_item(_robot->joints.wrist_pronation->menu());
    _menu->add_item(_robot->joints.hand->menu());
}

CompensationIMU::~CompensationIMU()
{
    _robot->joints.elbow_flexion->forward(0);
    _robot->joints.wrist_pronation->forward(0);
    stop_and_join();
}

void CompensationIMU::tare_IMU()
{
    //    double qFA[4];
    //    _robot->sensors.arm_imu->get_quat(qFA);
    //    qDebug("qarm: %lf; %lf; %lf; %lf", qFA[0], qFA[1], qFA[2], qFA[3]);
    //    _robot->sensors.fa_imu->get_quat(qFA);
    //    qDebug("qFA: %lf; %lf; %lf; %lf", qFA[0], qFA[1], qFA[2], qFA[3]);

    _robot->sensors.arm_imu->send_command_algorithm_init_then_tare();
    _robot->sensors.trunk_imu->send_command_algorithm_init_then_tare();

    _robot->sensors.fa_imu->send_command_algorithm_init_then_tare();

    debug("Wait ...");

    std::this_thread::sleep_for(std::chrono::seconds(6));
    _robot->user_feedback.buzzer->makeNoise(Buzzer::TRIPLE_BUZZ);
    //    _robot->sensors.arm_imu->get_quat(qFA);
    //    qDebug("qarm after tare: %lf; %lf; %lf; %lf", qFA[0], qFA[1], qFA[2], qFA[3]);
    //    _robot->sensors.fa_imu->get_quat(qFA);
    //    qDebug("qFA after tare: %lf; %lf; %lf; %lf", qFA[0], qFA[1], qFA[2], qFA[3]);
}

void CompensationIMU::receiveData()
{
    printf("in receiveData \n");
    while (_receiver.available()) {
        auto data = _receiver.receive();
        std::string buf;
        std::transform(data.begin(), data.end(), buf.begin(), [](std::byte b) { return static_cast<char>(b); });
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
        printf("lambdaW:%d\n", _lambdaW);
    }
}

void CompensationIMU::displayPin()
{
    int pin_down_value = _robot->btn2;
    int pin_up_value = _robot->btn1;
    debug() << "PinUp: " << pin_up_value;
    debug() << "PinDown: " << pin_down_value;
}

bool CompensationIMU::setup()
{
    //    if (_robot.elbow) {
    //        _robot.elbow->calibrate();
    //    }
    _robot->joints.wrist_pronation->set_encoder_position(0);

    std::string filename("compensationIMU");
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
    return true;
}

void CompensationIMU::loop(double dt, clock::time_point time)
{
    int init_cnt = 10;
    double timeWithDelta = (time - _start_time).count();

    receiveData();

    _robot->sensors.optitrack->update();
    optitrack_data_t data = _robot->sensors.optitrack->get_last_data();

    double debugData[10];

    if (_need_to_write_header) {
        _file << " time, pinUp, pinDown,";
        _file << " qBras.w, qBras.x, qBras.y, qBras.z, qTronc.w, qTronc.x, qTronc.y, qTronc.z,";
        _file << " qFA.w, qFA.x, qFA.y, qFA.z,";
        _file << " phi wrist, theta wrist, wrist angle, wristAngVel, lambdaW, thresholdW, wristEncoder,";
        _file << " nbRigidBodies";
        for (int i = 0; i < data.nRigidBodies; i++) {
            _file << ", ID, bTrackingValid, fError, qw, qx, qy, qz, x, y, z";
        }
        _file << "\r\n";
        _need_to_write_header = false;
    }

    /// WRIST
    double wristAngleEncoder = _robot->joints.wrist_pronation->read_encoder_position();

    double qBras[4], qTronc[4], qFA[4];
    _robot->sensors.arm_imu->get_quat(qBras);
    _robot->sensors.trunk_imu->get_quat(qTronc);
    _robot->sensors.fa_imu->get_quat(qFA);

    //    qDebug("qfa: %lf; %lf; %lf; %lf", qFA[0], qFA[1], qFA[2], qFA[3]);

    Eigen::Quaterniond qFA_record;
    qFA_record.w() = qFA[0];
    qFA_record.x() = qFA[1];
    qFA_record.y() = qFA[2];
    qFA_record.z() = qFA[3];

    if (_cnt == 0) {
        _lawimu.initialization();
    } else if (_cnt <= init_cnt) {
        _lawimu.initialPositions(qFA_record, _cnt, init_cnt);
    } else {
        _lawimu.rotationMatrices(qFA_record);
        _lawimu.controlLawWrist(_lambdaW, _thresholdW);

        if (_lawimu.returnWristVel_deg() > 0) {
            _robot->joints.wrist_pronation->move_to(350, _lawimu.returnWristVel_deg());
        } else if (_lawimu.returnWristVel_deg() < 0) {
            _robot->joints.wrist_pronation->move_to(-350, -_lawimu.returnWristVel_deg());
        } else if (_lawimu.returnWristVel_deg() == 0) {
            _robot->joints.wrist_pronation->forward(0);
        }

        if (_cnt % 50 == 0) {
            _lawimu.displayData();
            // qDebug("lambdaW: %d", _lambdaW);
            //            printf("lambdaW: %d\n", _lambdaW);
        }
    }

    _lawimu.writeDebugData(debugData);

    int pin_down_value = _robot->btn2;
    int pin_up_value = _robot->btn1;

    _file << timeWithDelta << ' ' << pin_down_value << ' ' << pin_up_value;
    _file << ' ' << qBras[0] << ' ' << qBras[1] << ' ' << qBras[2] << ' ' << qBras[3] << ' ' << qTronc[0] << ' ' << qTronc[1] << ' ' << qTronc[2] << ' ' << qTronc[3];
    _file << ' ' << qFA[0] << ' ' << qFA[1] << ' ' << qFA[2] << ' ' << qFA[3];
    _file << ' ' << debugData[0] << ' ' << debugData[1] << ' ' << debugData[2] << ' ' << debugData[3];
    _file << ' ' << _lambdaW << ' ' << _thresholdW << ' ' << wristAngleEncoder;
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

void CompensationIMU::cleanup()
{
    //    _robot.elbow->forward(0);
    _robot->joints.wrist_pronation->forward(0);
    _file.close();
}
