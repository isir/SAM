#include "demo_imu.h"
#include "utils/check_ptr.h"
#include "utils/log/log.h"
#include "wiringPi.h"
#include <filesystem>

DemoIMU::DemoIMU(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("Demo IMU", 0.01)
    , _robot(robot)
    , _lambdaW(2)
    , _thresholdW(4. * M_PI / 180.)
    , _pin_up(24)
    , _pin_down(22)
{
    if (!check_ptr(_robot->joints.wrist_pronation, _robot->sensors.red_imu)) {
        throw std::runtime_error("Compensation IMU Control is missing components");
    }

    _menu->set_description("DemoIMU");
    _menu->set_code("demoimu");
    _menu->add_item("tare", "Tare IMUs", [this](std::string) { this->tare_IMU(); });

    _menu->add_item(_robot->joints.wrist_pronation->menu());
    _menu->add_item(_robot->joints.hand->menu());

    pullUpDnControl(_pin_up, PUD_UP);
    pullUpDnControl(_pin_down, PUD_UP);
}

DemoIMU::~DemoIMU()
{
    _robot->joints.wrist_pronation->forward(0);
    stop_and_join();
}

void DemoIMU::tare_IMU()
{ /// IMU to use = red IMU
    _robot->sensors.red_imu->send_command_algorithm_init_then_tare();

    debug() << "Wait ...";

    std::this_thread::sleep_for(std::chrono::seconds(5));
    _robot->user_feedback.buzzer->makeNoise(Buzzer::TRIPLE_BUZZ);
}

void DemoIMU::displayPin()
{
    int pin_down_value = digitalRead(_pin_down);
    int pin_up_value = digitalRead(_pin_up);
    debug() << "PinUp: " << pin_up_value;
    debug() << "PinDown: " << pin_down_value;
}

bool DemoIMU::setup()
{
    _robot->joints.wrist_pronation->set_encoder_position(0);
    _cnt = 0;
    _start_time = clock::now();
    return true;
}

void DemoIMU::loop(double dt, clock::time_point time)
{
    int init_cnt = 10;
    double timeWithDelta = (time - _start_time).count();

    double qFA[4];
    _robot->sensors.red_imu->get_quat(qFA);

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

        if (_lawimu.returnWristVel_deg() < 0)
            _robot->joints.wrist_pronation->set_velocity_safe(_lawimu.returnWristVel_deg());
        //            _robot->joints.wrist_pronation->move_to(6000, _lawimu.returnWristVel_deg() * 100, 6000, 35000);
        else if (_lawimu.returnWristVel_deg() > 0)
            _robot->joints.wrist_pronation->set_velocity_safe(_lawimu.returnWristVel_deg());
        //            _robot->joints.wrist_pronation->move_to(6000, -_lawimu.returnWristVel_deg() * 100, 6000, -35000);
        else if (_lawimu.returnWristVel_deg() == 0) {
            _robot->joints.wrist_pronation->forward(0);
        }
    }
    /// OPEN-LOOP WITH PUSH BUTTONS
    static int prev_pin_up_value = 1, prev_pin_down_value = 1;
    int pin_down_value = digitalRead(_pin_down);
    int pin_up_value = digitalRead(_pin_up);
    /// WRIST
    if (pin_down_value == 0 && prev_pin_down_value == 1) {
        _robot->joints.wrist_pronation->move_to(350, 50);
    } else if (pin_up_value == 0 && prev_pin_up_value == 1) {
        _robot->joints.wrist_pronation->move_to(-350, 50);
    } else if ((pin_down_value == 1 && pin_up_value == 1) && (prev_pin_down_value == 0 || prev_pin_up_value == 0)) {
        _robot->joints.wrist_pronation->forward(0);
    }

    prev_pin_down_value = pin_down_value;
    prev_pin_up_value = pin_up_value;

    ++_cnt;
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()).count() << "ms" << std::endl;
}

void DemoIMU::cleanup()
{
    //    _robot.elbow->forward(0);
    _robot->joints.wrist_pronation->forward(0);
}
