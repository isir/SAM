#include "voluntary_control.h"
#include "peripherals/roboclaw/factory.h"
#include "utils/check_ptr.h"
#include "utils/log/log.h"
#include <filesystem>
#include <wiringPi.h>

VoluntaryControl::VoluntaryControl(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("Voluntary control")
    , _robot(robot)
{
    if (!check_ptr(_robot->joints.elbow_flexion, _robot->joints.wrist_pronation)) {
        throw std::runtime_error("Volontary Control is missing components");
    }

    _pin_up = 24;
    _pin_down = 22;
    set_period(0.01);

    _menu->set_description("Voluntary Control");
    _menu->set_code("vc");
    _menu->add_item(_robot->joints.elbow_flexion->menu());

    pullUpDnControl(_pin_up, PUD_UP);
    pullUpDnControl(_pin_down, PUD_UP);
}

VoluntaryControl::~VoluntaryControl()
{
    _robot->joints.elbow_flexion->forward(0);
    _robot->joints.wrist_pronation->forward(0);
    stop_and_join();
}

bool VoluntaryControl::setup()
{
    _robot->joints.wrist_pronation->set_encoder_position(0);
    std::string filename("voluntary");
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
    return true;
}

void VoluntaryControl::loop(double, clock::time_point)
{
    static int prev_pin_up_value = 1, prev_pin_down_value = 1;
    int pin_down_value = digitalRead(_pin_down);
    int pin_up_value = digitalRead(_pin_up);

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

    optitrack_data_t data = _robot->sensors.optitrack->get_last_data();
    if (_need_to_write_header) {
        //        _file.write("period, btnUp, btnDown, beta");
        _file << "period, btnUp, btnDown, wristAngle";
        for (int i = 0; i < data.nRigidBodies; i++) {
            _file << ", ID, bTrackingValid, fError, qw, qx, qy, qz, x, y, z";
        }
        _file << std::endl;
        _need_to_write_header = false;
    }

    //ts.setPadChar(' ');
    //    ts << return_period() << ' ' << pin_up_value << ' ' << pin_down_value << ' ' << beta;
    _file << period() << ' ' << pin_up_value << ' ' << pin_down_value << ' ' << wristAngle;
    for (unsigned int i = 0; i < data.nRigidBodies; i++) {
        _file << ' ' << data.rigidBodies[i].ID << ' ' << data.rigidBodies[i].bTrackingValid << ' ' << data.rigidBodies[i].fError;
        _file << ' ' << data.rigidBodies[i].qw << ' ' << data.rigidBodies[i].qx << ' ' << data.rigidBodies[i].qy << ' ' << data.rigidBodies[i].qz;
        _file << ' ' << data.rigidBodies[i].x << ' ' << data.rigidBodies[i].y << ' ' << data.rigidBodies[i].z;
    }
    _file << std::endl;
}

void VoluntaryControl::cleanup()
{
    //_robot.elbow->forward(0);
    _robot->joints.wrist_pronation->forward(0);
    _file.close();
}
