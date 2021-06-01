#include "recorddata.h"
#include "utils/check_ptr.h"
#include "utils/log/log.h"
#include <filesystem>
#include <iostream>

RecordData::RecordData(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("Record data", 0.01)
    , _robot(robot)
{
    if (!check_ptr(_robot->sensors.white_imu, _robot->sensors.red_imu)) {
        throw std::runtime_error("Record data is missing components");
    }

    _menu->set_description("Record IMU and optitrack data");
    _menu->set_code("rec");
    _menu->add_item("tare", "Tare IMUs", [this](std::string) { this->tare_IMU(); });
    _menu->add_item("analog", "Show IMU Analog data", [this](std::string) { this->analog_IMU(); });
}

RecordData::~RecordData()
{
    stop_and_join();
}

void RecordData::analog_IMU()
{
    double a[8];
    _robot->sensors.white_imu->get_analog(a);
}

void RecordData::tare_IMU()
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

bool RecordData::setup()
{

    // OPEN AND NAME DATA FILE
    std::string _filename("data");
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
    _start_time = clock::now();

    return true;
}

void RecordData::loop(double, clock::time_point time)
{

    double timeWithDelta = (time - _start_time).count();

    _robot->sensors.optitrack->update();
    optitrack_data_t data = _robot->sensors.optitrack->get_last_data();

    double debugData[40];

    /// WRITE FILE HEADERS
    if (_need_to_write_header) {
        _file << " time,";
        _file << " qWhite.w, qWhite.x, qWhite.y, qWhite.z, qTronc.w, qTronc.x, qTronc.y, qTronc.z,";
        _file << " qYellow.w, qYellow.x, qYellow.y, qYellow.z,";
        _file << " to complete,";
        _file << "\r\n";
        _need_to_write_header = false;
    }

    /// IMU - Initialization data
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
        _qArm.w() = qWhite[0];
        _qArm.x() = qWhite[1];
        _qArm.y() = qWhite[2];
        _qArm.z() = qWhite[3];
        //        debug() << "qwhite: " << qWhite[0] << "; " << qWhite[1] << "; " << qWhite[2] << "; " << qWhite[3];
    }
    if (_robot->sensors.red_imu) {
        _robot->sensors.red_imu->get_quat(qRed);
        _qTrunk.w() = qRed[0];
        _qTrunk.x() = qRed[1];
        _qTrunk.y() = qRed[2];
        _qTrunk.z() = qRed[3];
        //        debug() << "qRed: " << _qHip.w() << "; " << _qHip.x() << "; " << _qHip.y() << "; " << _qHip.z();
    }
    if (_robot->sensors.yellow_imu) {
        _robot->sensors.yellow_imu->get_quat(qYellow);
        //        debug() << "qyellow: " << qYellow[0] << "; " << qYellow[1] << "; " << qYellow[2] << "; " << qYellow[3];
    }

    /// WRITE DATA
    _file << timeWithDelta;
    _file << ' ' << qWhite[0] << ' ' << qWhite[1] << ' ' << qWhite[2] << ' ' << qWhite[3] << ' ' << qRed[0] << ' ' << qRed[1] << ' ' << qRed[2] << ' ' << qRed[3];
    _file << ' ' << qYellow[0] << ' ' << qYellow[1] << ' ' << qYellow[2] << ' ' << qYellow[3];
    for (unsigned int i = 0; i < data.nRigidBodies; i++) {
        _file << ' ' << data.rigidBodies[i].ID << ' ' << data.rigidBodies[i].bTrackingValid << ' ' << data.rigidBodies[i].fError;
        _file << ' ' << data.rigidBodies[i].qw << ' ' << data.rigidBodies[i].qx << ' ' << data.rigidBodies[i].qy << ' ' << data.rigidBodies[i].qz;
        _file << ' ' << data.rigidBodies[i].x << ' ' << data.rigidBodies[i].y << ' ' << data.rigidBodies[i].z;
    }

    _file << std::endl;

    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()).count() << "ms" << std::endl;
}

void RecordData::cleanup()
{
    _file.close();
}
