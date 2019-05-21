#include "roboclaw.h"
#include "cast_helper.h"
#include "factory.h"

#include <QDebug>
#include <QTime>

RC::RoboClaw::RoboClaw()
    : _address(0x80)
    , _channel(M1)
{
}

RC::RoboClaw::~RoboClaw()
{
}

void RC::RoboClaw::init(QString port_name, unsigned int baudrate, quint8 address, Channel channel)
{
    _serial_port = Factory::get(port_name, baudrate);
    _address = address;
    _channel = channel;
}

void RC::RoboClaw::forward(quint8 value)
{
    quint8 function_code = _channel == Channel::M1 ? 0 : 4;
    send(Message(_address, function_code, CastHelper::from(value), "\\xff"));
}

void RC::RoboClaw::backward(quint8 value)
{
    quint8 function_code = _channel == Channel::M1 ? 1 : 5;
    send(Message(_address, function_code, CastHelper::from(value), "\\xff"));
}

qint32 RC::RoboClaw::read_encoder_position()
{
    quint8 function_code = _channel == Channel::M1 ? 16 : 17;
    return CastHelper::to<qint32>(send(Message(_address, function_code, QByteArray(), "(.{5}).{2}", false)));
}

qint32 RC::RoboClaw::read_encoder_speed()
{
    quint8 function_code = _channel == Channel::M1 ? 30 : 31;
    return CastHelper::to<qint32>(send(Message(_address, function_code, QByteArray(), "(.{5}).{2}", false)));
}

void RC::RoboClaw::set_velocity(qint32 value)
{
    quint8 function_code = _channel == Channel::M1 ? 35 : 36;
    send(Message(_address, function_code, CastHelper::from(value), "(\\xff)"));
}

QString RC::RoboClaw::read_firmware_version()
{
    return QString::fromLatin1(send(Message(_address, 21, QByteArray(), "(.{,48})\\n\\x00.{2}", false)));
}

void RC::RoboClaw::set_encoder_position(qint32 value)
{
    quint8 function_code = _channel == Channel::M1 ? 22 : 23;
    send(Message(_address, function_code, CastHelper::from(static_cast<quint32>(value)), "(\\xff)"));
}

double RC::RoboClaw::read_main_battery_voltage()
{
    return .1 * CastHelper::to<quint16>(send(Message(_address, 24, QByteArray(), "(.{2}).{2}")));
}

double RC::RoboClaw::read_current()
{
    QByteArray buf = send(Message(_address, 49, QByteArray(), "(.{4}).{2}", false));
    if (_channel == M1)
        return CastHelper::to<qint16>(buf) / 100.;
    else {
        return CastHelper::to<qint16>(buf.mid(2)) / 100.;
    }
}

void RC::RoboClaw::set_velocity_pid(velocity_pid_params_t params)
{
    quint8 function_code = _channel == Channel::M1 ? 28 : 29;
    send(Message(_address, function_code, CastHelper::from(static_cast<quint32>(qRound(65536 * params.d))) + CastHelper::from(static_cast<quint32>(qRound(65536 * params.p))) + CastHelper::from(static_cast<quint32>(qRound(65536 * params.i))) + CastHelper::from(params.qpps), "(\\xff)"));
}

RC::velocity_pid_params_t RC::RoboClaw::read_velocity_pid()
{
    quint8 function_code = _channel == Channel::M1 ? 55 : 56;
    velocity_pid_params_t ret;
    QByteArray buf = send(Message(_address, function_code, QByteArray(), "(.{16}).{2}", false));
    ret.p = static_cast<float>(CastHelper::to<quint32>(buf) / 65536.);
    buf.remove(0, 4);
    ret.i = static_cast<float>(CastHelper::to<quint32>(buf) / 65536.);
    buf.remove(0, 4);
    ret.d = static_cast<float>(CastHelper::to<quint32>(buf) / 65536.);
    buf.remove(0, 4);
    ret.qpps = CastHelper::to<quint32>(buf);
    return ret;
}

void RC::RoboClaw::set_position_pid(position_pid_params_t params)
{
    quint8 function_code = _channel == Channel::M1 ? 61 : 62;
    send(Message(_address, function_code, CastHelper::from(static_cast<quint32>(qRound(1024 * params.d))) + CastHelper::from(static_cast<quint32>(qRound(1024 * params.p))) + CastHelper::from(static_cast<quint32>(qRound(1024 * params.i))) + CastHelper::from(params.i_max) + CastHelper::from(params.deadzone) + CastHelper::from(params.min_pos) + CastHelper::from(params.max_pos), "(\\xff)"));
}

RC::position_pid_params_t RC::RoboClaw::read_position_pid()
{
    quint8 function_code = _channel == Channel::M1 ? 63 : 64;
    position_pid_params_t ret;
    QByteArray buf = send(Message(_address, function_code, QByteArray(), "(.{28}).{2}", false));
    ret.p = static_cast<float>(CastHelper::to<quint32>(buf) / 1024.);
    buf.remove(0, 4);
    ret.i = static_cast<float>(CastHelper::to<quint32>(buf) / 1024.);
    buf.remove(0, 4);
    ret.d = static_cast<float>(CastHelper::to<quint32>(buf) / 1024.);
    buf.remove(0, 4);
    ret.i_max = CastHelper::to<quint32>(buf);
    buf.remove(0, 4);
    ret.deadzone = CastHelper::to<quint32>(buf);
    buf.remove(0, 4);
    ret.min_pos = CastHelper::to<qint32>(buf);
    buf.remove(0, 4);
    ret.max_pos = CastHelper::to<qint32>(buf);
    return ret;
}

void RC::RoboClaw::move_to(quint32 accel, quint32 speed, quint32 decel, qint32 pos)
{
    quint8 function_code = _channel == Channel::M1 ? 65 : 66;
    send(Message(_address, function_code, CastHelper::from(accel) + CastHelper::from(speed) + CastHelper::from(decel) + CastHelper::from(pos) + CastHelper::from<quint8>(1), "\\xff"));
}

QByteArray RC::RoboClaw::send(const Message& msg)
{
    static const int to = 20;
    QByteArray ret;
    QTime t;
    QRegExp rx(msg.regexp());

    _serial_port->take_ownership();
    _serial_port->read_all();

    t.start();

    _serial_port->write(msg.data());

    while (true) {
        ret.append(_serial_port->read_all());
        int pos = rx.indexIn(QString::fromLatin1(ret, ret.length()));
        if (pos > -1 && rx.captureCount() > 0) {
            ret = ret.mid(rx.pos(1), rx.cap(1).length());
            break;
        }

        if (t.elapsed() >= to) {
            throw std::runtime_error(std::string("Request timed out: ") + msg.toString().toStdString() + " - Pattern is [" + msg.regexp().toStdString() + "] - Rx buffer contains [" + ret.toHex().toStdString() + "]");
        }
    }

    _serial_port->release_ownership();
    return ret;
}
