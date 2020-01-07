#include "actuator.h"
#include "utils/log/log.h"
#include <cmath>

Actuator::Actuator(std::string name)
    : RoboClaw::RoboClaw()
    , MenuUser("", "", [this]() { on_exit(); })
    , _name(name)
    , _connected(false)
    , _calibrated(false)
    , _incs_per_deg(0)
    , _acc(0)
{
    _menu->add_item("f", "Forward (0-127)", [this](std::string args) { if(args.length() == 0) args = "20"; forward(static_cast<uint8_t>(std::stoi(args))); });
    _menu->add_item("b", "Backward (0-127)", [this](std::string args) { if (args.length() == 0) args = "20"; backward(static_cast<uint8_t>(std::stoi(args))); });
    _menu->add_item("pc", "Print current", [this](std::string) { info() << "Current:" << read_current() << "A"; });
    _menu->add_item("fw", "Print firmware version", [this](std::string) { info() << read_firmware_version(); });
    _menu->add_item("es", "Print encoder speed", [this](std::string) { info() << "Speed:" << read_encoder_speed() << "steps/s"; });
    _menu->add_item("s", "Stop", [this](std::string) { forward(0); });
    _menu->add_item("calib", "Calibrate", [this](std::string) { calibrate(); });
    _menu->add_item("e", "Read encoder", [this](std::string) { info() << "Position:" << read_encoder_position() << "steps"; });
    _menu->add_item("a", "Read angle", [this](std::string) { info() << "Angle:" << pos() << " deg"; });
    _menu->add_item("g", "Go to", [this](std::string args) {if (args.length() > 0) move_to(std::stod(args), 10); });
    _menu->add_item("v", "Set velocity (deg/s)", [this](std::string args) { if (args.length() > 0) args = "0"; set_velocity_safe(std::stod(args)); });
    _menu->add_item("z", "Set encoder zero", [this](std::string) { set_encoder_position(0); });
}

double Actuator::pos()
{
    return static_cast<double>(read_encoder_position()) / _incs_per_deg;
}

void Actuator::move_to(double deg, double speed, bool block)
{
    if (!_calibrated) {
        warning() << "Not calibrated...";
        return;
    }

    int32_t target = static_cast<int32_t>(deg * _incs_per_deg);
    uint32_t velocity = static_cast<uint32_t>(speed * _incs_per_deg);

    if (velocity == 0)
        velocity = 1;

    RoboClaw::move_to(_acc, velocity, _acc, target);

    if (block) {
        int threshold = 10000;
        do {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } while ((std::abs(target - read_encoder_position()) > threshold));
    }
}

void Actuator::set_velocity(double deg_s)
{
    RoboClaw::RoboClaw::set_velocity(static_cast<int32_t>(std::round(deg_s * _incs_per_deg)));
}

void Actuator::set_velocity_safe(double deg_s)
{
    if (!_calibrated) {
        warning() << "Not calibrated...";
        return;
    }

    move_to(deg_s > 0 ? _max_angle : _min_angle, std::fabs(deg_s));
}

void Actuator::calibrate(double velocity_deg_s, double final_pos, double velocity_threshold_deg_s, bool use_velocity_control)
{
    double calib_velocity_threshold = velocity_threshold_deg_s * _incs_per_deg;

    if (use_velocity_control) {
        set_velocity(velocity_deg_s);
    } else {
        if (velocity_deg_s < 0) {
            backward(static_cast<uint8_t>(std::fabs(velocity_deg_s)));
        } else {
            forward(static_cast<uint8_t>(velocity_deg_s));
        }
    }

    auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() < 500)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    while (std::abs(read_encoder_speed()) > calib_velocity_threshold) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    set_encoder_position(static_cast<int32_t>(std::round(final_pos * _incs_per_deg)));
    forward(0);
    _calibrated = true;

    RC::position_pid_params_t p_params = read_position_pid();
    p_params.min_pos = static_cast<int32_t>(_min_angle * _incs_per_deg);
    p_params.max_pos = static_cast<int32_t>(_max_angle * _incs_per_deg);

    set_position_pid(p_params);
}

void Actuator::set_params_limits(double lower_limit_deg, double upper_limit_deg)
{
    _min_angle = lower_limit_deg;
    _max_angle = upper_limit_deg;
}

void Actuator::set_params_technical(unsigned int incs_per_deg, unsigned int deg_s2)
{
    _incs_per_deg = incs_per_deg;
    _acc = deg_s2 * _incs_per_deg;
}

void Actuator::set_params_velocity(float kp, float ki, float kd, unsigned int qpps)
{
    RC::velocity_pid_params_t v_params = {};
    v_params.p = kp;
    v_params.i = ki;
    v_params.d = kd;
    v_params.qpps = qpps;
    set_velocity_pid(v_params);
}

void Actuator::set_params_position(float kp, float ki, float kd, uint32_t i_max, uint32_t deadzone, int32_t lower_limit, int32_t upper_limit)
{
    RC::position_pid_params_t p_params = {};
    p_params.p = kp;
    p_params.i = ki;
    p_params.d = kd;
    p_params.i_max = i_max;
    p_params.deadzone = deadzone;
    p_params.min_pos = lower_limit * static_cast<int32_t>(_incs_per_deg);
    p_params.max_pos = upper_limit * static_cast<int32_t>(_incs_per_deg);
    set_position_pid(p_params);
}

void Actuator::on_exit()
{
    forward(0);
}
