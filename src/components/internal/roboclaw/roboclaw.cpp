#include "roboclaw.h"
#include "cast_helper.h"
#include "factory.h"
#include <cmath>

#define ff_answer std::make_shared<Answer::ExactMatch>(std::vector<std::byte>(1, std::byte { 0xff }))
#define crc_answer std::make_shared<Answer::EndsWithCRC>()

RC::RoboClaw::RoboClaw()
    : _address(0x80)
    , _channel(M1)
{
}

RC::RoboClaw::~RoboClaw()
{
}

void RC::RoboClaw::init(std::string port_name, unsigned int baudrate, uint8_t address, Channel channel)
{
    _serial_port = Factory::get(port_name, baudrate);
    _address = address;
    _channel = channel;
}

void RC::RoboClaw::forward(uint8_t value)
{
    std::vector<std::byte>(1, std::byte { 0xff });
    send(Message(_address, get_fn_code(0, 4), ff_answer, CastHelper::from(value)));
}

void RC::RoboClaw::backward(uint8_t value)
{
    send(Message(_address, get_fn_code(1, 5), ff_answer, CastHelper::from(value)));
}

int32_t RC::RoboClaw::read_encoder_position()
{
    return CastHelper::to<int32_t>(send(Message(_address, get_fn_code(16, 17), crc_answer, std::vector<std::byte>(), false)));
}

int32_t RC::RoboClaw::read_encoder_speed()
{
    return CastHelper::to<int32_t>(send(Message(_address, get_fn_code(30, 31), crc_answer, std::vector<std::byte>(), false)));
}

void RC::RoboClaw::set_velocity(int32_t value)
{
    send(Message(_address, get_fn_code(35, 36), ff_answer, CastHelper::from(value)));
}

std::string RC::RoboClaw::read_firmware_version()
{
    std::vector<std::byte> ans = send(Message(_address, 21, crc_answer, std::vector<std::byte>(), false));
    std::string ret;
    for (auto b : ans) {
        ret.push_back(static_cast<char>(b));
    }
    return ret.substr(0, ret.find('\n'));
}

void RC::RoboClaw::set_encoder_position(int32_t value)
{
    send(Message(_address, get_fn_code(22, 23), ff_answer, CastHelper::from(static_cast<uint32_t>(value))));
}

double RC::RoboClaw::read_main_battery_voltage()
{
    return .1 * CastHelper::to<uint16_t>(send(Message(_address, 24, crc_answer, std::vector<std::byte>())));
}

double RC::RoboClaw::read_current()
{
    std::vector<std::byte> buf = send(Message(_address, 49, crc_answer, std::vector<std::byte>(), false));
    if (_channel == M1)
        return CastHelper::to<int16_t>(buf) / 100.;
    else {
        return CastHelper::to<int16_t>(std::vector<std::byte>(buf.begin() + 2, buf.end())) / 100.;
    }
}

void RC::RoboClaw::set_velocity_pid(velocity_pid_params_t params)
{
    std::vector<std::byte> tmp, payload;
    tmp = CastHelper::from(static_cast<uint32_t>(std::round(65536 * params.d)));
    payload.insert(payload.end(), tmp.begin(), tmp.end());
    tmp = CastHelper::from(static_cast<uint32_t>(std::round(65536 * params.p)));
    payload.insert(payload.end(), tmp.begin(), tmp.end());
    tmp = CastHelper::from(static_cast<uint32_t>(std::round(65536 * params.i)));
    payload.insert(payload.end(), tmp.begin(), tmp.end());
    tmp = CastHelper::from(params.qpps);
    payload.insert(payload.end(), tmp.begin(), tmp.end());
    send(Message(_address, get_fn_code(28, 29), ff_answer, payload));
}

RC::velocity_pid_params_t RC::RoboClaw::read_velocity_pid()
{
    velocity_pid_params_t ret;
    std::vector<std::byte> buf = send(Message(_address, get_fn_code(55, 56), crc_answer, std::vector<std::byte>(), false));
    ret.p = static_cast<float>(CastHelper::to<uint32_t>(buf) / 65536.);
    buf.erase(buf.begin(), buf.begin() + 4);
    ret.i = static_cast<float>(CastHelper::to<uint32_t>(buf) / 65536.);
    buf.erase(buf.begin(), buf.begin() + 4);
    ret.d = static_cast<float>(CastHelper::to<uint32_t>(buf) / 65536.);
    buf.erase(buf.begin(), buf.begin() + 4);
    ret.qpps = CastHelper::to<uint32_t>(buf);
    return ret;
}

void RC::RoboClaw::set_position_pid(position_pid_params_t params)
{
    std::vector<std::byte> tmp, payload;
    tmp = CastHelper::from(static_cast<uint32_t>(std::round(1024 * params.d)));
    payload.insert(payload.end(), tmp.begin(), tmp.end());
    tmp = CastHelper::from(static_cast<uint32_t>(std::round(1024 * params.p)));
    payload.insert(payload.end(), tmp.begin(), tmp.end());
    tmp = CastHelper::from(static_cast<uint32_t>(std::round(1024 * params.i)));
    payload.insert(payload.end(), tmp.begin(), tmp.end());
    tmp = CastHelper::from(params.i_max);
    payload.insert(payload.end(), tmp.begin(), tmp.end());
    tmp = CastHelper::from(params.deadzone);
    payload.insert(payload.end(), tmp.begin(), tmp.end());
    tmp = CastHelper::from(params.min_pos);
    payload.insert(payload.end(), tmp.begin(), tmp.end());
    tmp = CastHelper::from(params.max_pos);
    payload.insert(payload.end(), tmp.begin(), tmp.end());
    send(Message(_address, get_fn_code(61, 62), ff_answer, payload));
}

RC::position_pid_params_t RC::RoboClaw::read_position_pid()
{
    position_pid_params_t ret;
    std::vector<std::byte> buf = send(Message(_address, get_fn_code(63, 64), crc_answer, std::vector<std::byte>(), false));
    ret.p = static_cast<float>(CastHelper::to<uint32_t>(buf) / 1024.);
    buf.erase(buf.begin(), buf.begin() + 4);
    ret.i = static_cast<float>(CastHelper::to<uint32_t>(buf) / 1024.);
    buf.erase(buf.begin(), buf.begin() + 4);
    ret.d = static_cast<float>(CastHelper::to<uint32_t>(buf) / 1024.);
    buf.erase(buf.begin(), buf.begin() + 4);
    ret.i_max = CastHelper::to<uint32_t>(buf);
    buf.erase(buf.begin(), buf.begin() + 4);
    ret.deadzone = CastHelper::to<uint32_t>(buf);
    buf.erase(buf.begin(), buf.begin() + 4);
    ret.min_pos = CastHelper::to<int32_t>(buf);
    buf.erase(buf.begin(), buf.begin() + 4);
    ret.max_pos = CastHelper::to<int32_t>(buf);
    return ret;
}

void RC::RoboClaw::move_to(uint32_t accel, uint32_t speed, uint32_t decel, int32_t pos)
{
    std::vector<std::byte> tmp, payload;
    tmp = CastHelper::from(accel);
    payload.insert(payload.end(), tmp.begin(), tmp.end());
    tmp = CastHelper::from(speed);
    payload.insert(payload.end(), tmp.begin(), tmp.end());
    tmp = CastHelper::from(decel);
    payload.insert(payload.end(), tmp.begin(), tmp.end());
    tmp = CastHelper::from(pos);
    payload.insert(payload.end(), tmp.begin(), tmp.end());
    tmp = CastHelper::from<uint8_t>(1);
    payload.insert(payload.end(), tmp.begin(), tmp.end());
    send(Message(_address, get_fn_code(65, 66), ff_answer, payload));
}

std::vector<std::byte> RC::RoboClaw::send(const Message& msg)
{
    static const int to = 100;
    std::vector<std::byte> ret;

    _serial_port->take_ownership();
    _serial_port->read_all();

    auto start = std::chrono::steady_clock::now();

    _serial_port->write(msg.data());

    while (true) {
        std::vector<std::byte> tmp = _serial_port->read_all();
        ret.insert(ret.end(), tmp.begin(), tmp.end());

        if (msg.answer()->try_match(ret, msg.data())) {
            break;
        }

        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();

        if (elapsed_ms >= to) {
            _serial_port->release_ownership();
            throw std::runtime_error("Request timed out: " + msg.to_string());
        }
    }

    _serial_port->release_ownership();
    return msg.answer()->format(ret);
}
