#include "client.h"
#include "casthelper.h"

#include <QEventLoop>
#include <QTimer>
#include <iostream>

RoboClaw::Client::Client(quint8 address, Channel channel) : _address(address), _channel(channel)
{

}

RoboClaw::Client::~Client() {

}

void RoboClaw::Client::set_address(quint8 address, Channel channel) {
    _address = address;
    _channel = channel;
}

void RoboClaw::Client::on_answer_received(QByteArray data) {
    emit answer_received_internal(data);
}

void RoboClaw::Client::forward(quint8 value) {
    quint8 function_code = _channel == Channel::M1 ? 0 : 4;
    send(Message(_address,function_code,CastHelper::from(value),"\\xff"));
}

void RoboClaw::Client::backward(quint8 value) {
    quint8 function_code = _channel == Channel::M1 ? 1 : 5;
    send(Message(_address,function_code,CastHelper::from(value),"\\xff"));
}

qint32 RoboClaw::Client::read_encoder_position() {
    quint8 function_code = _channel == Channel::M1 ? 16 : 17;
    return CastHelper::to<qint32>(send(Message(_address,function_code,QByteArray(),"(.{5}).{2}"), true));
}

qint32 RoboClaw::Client::read_encoder_speed() {
    quint8 function_code = _channel == Channel::M1 ? 30 : 31;
    return CastHelper::to<qint32>(send(Message(_address,function_code,QByteArray(),"(.{5}).{2}"), true));
}

QString RoboClaw::Client::read_firmware_version() {
    return QString::fromLatin1(send(Message(_address,21,QByteArray(),"(.{,48})\\n\\x00.{2}"), true));
}

void RoboClaw::Client::set_encoder_position(qint32 value) {
    quint8 function_code = _channel == Channel::M1 ? 22 : 23;
    send(Message(_address,function_code,CastHelper::from(static_cast<quint32>(value)),"(\\xff)"), true);
}

double RoboClaw::Client::read_main_battery_voltage() {
    return .1*CastHelper::to<quint16>(send(Message(_address,24,QByteArray(),"(.{2}).{2}"), true));
}

double RoboClaw::Client::read_current() {
    QByteArray buf = send(Message(_address,49,QByteArray(),"(.{4}).{2}"), true);
    std::cout << "read_current " << buf.toHex().toStdString() << " " << buf.length() << std::endl;
    if(_channel == M1)
        return CastHelper::to<qint16>(buf)/100.;
    else {
        return CastHelper::to<qint16>(buf.mid(2))/100.;
    }
}

void RoboClaw::Client::set_velocity_pid(RoboClaw::velocity_pid_params_t params) {
    quint8 function_code = _channel == Channel::M1 ? 28 : 29;
    send(Message(_address,function_code,CastHelper::from(static_cast<quint32>(qRound(65536*params.d))) + CastHelper::from(static_cast<quint32>(qRound(65536*params.p))) + CastHelper::from(static_cast<quint32>(qRound(65536*params.i))) + CastHelper::from(params.qpps),"(\\xff)"),true);
}

RoboClaw::velocity_pid_params_t RoboClaw::Client::read_velocity_pid() {
    quint8 function_code = _channel == Channel::M1 ? 55 : 56;
    RoboClaw::velocity_pid_params_t ret;
    QByteArray buf = send(Message(_address,function_code,QByteArray(),"(.{16}).{2}"), true);
    ret.p = static_cast<float>(CastHelper::to<quint32>(buf)/65536.); buf.remove(0,4);
    ret.i = static_cast<float>(CastHelper::to<quint32>(buf)/65536.); buf.remove(0,4);
    ret.d = static_cast<float>(CastHelper::to<quint32>(buf)/65536.); buf.remove(0,4);
    ret.qpps = CastHelper::to<quint32>(buf);
    return ret;
}

void RoboClaw::Client::set_position_pid(RoboClaw::position_pid_params_t params) {
    quint8 function_code = _channel == Channel::M1 ? 61 : 62;
    send(Message(_address,function_code,CastHelper::from(static_cast<quint32>(qRound(1024*params.d))) + CastHelper::from(static_cast<quint32>(qRound(1024*params.p))) + CastHelper::from(static_cast<quint32>(qRound(1024*params.i))) + CastHelper::from(params.i_max) + CastHelper::from(params.deadzone) + CastHelper::from(params.min_pos) + CastHelper::from(params.max_pos),"(\\xff)"),true);
}

RoboClaw::position_pid_params_t RoboClaw::Client::read_position_pid() {
    quint8 function_code = _channel == Channel::M1 ? 63 : 64;
    RoboClaw::position_pid_params_t ret;
    QByteArray buf = send(Message(_address,function_code,QByteArray(),"(.{28}).{2}"), true);
    ret.p = static_cast<float>(CastHelper::to<quint32>(buf)/1024.); buf.remove(0,4);
    ret.i = static_cast<float>(CastHelper::to<quint32>(buf)/1024.); buf.remove(0,4);
    ret.d = static_cast<float>(CastHelper::to<quint32>(buf)/1024.); buf.remove(0,4);
    ret.i_max = CastHelper::to<quint32>(buf); buf.remove(0,4);
    ret.deadzone = CastHelper::to<quint32>(buf); buf.remove(0,4);
    ret.min_pos = CastHelper::to<qint32>(buf); buf.remove(0,4);
    ret.max_pos = CastHelper::to<qint32>(buf);
    return ret;
}

void RoboClaw::Client::move_to(quint32 accel, quint32 speed, quint32 decel, qint32 pos) {
    quint8 function_code = _channel == Channel::M1 ? 65 : 66;
    send(Message(_address,function_code,CastHelper::from(accel) + CastHelper::from(speed) + CastHelper::from(decel) + CastHelper::from(pos) + CastHelper::from<quint8>(1),"\\xff"));
}

QByteArray RoboClaw::Client::send(const Message& msg, bool wait_for_answer) {
    QByteArray ret;

    if(wait_for_answer) {
        QEventLoop el;

        QTimer timer;
        timer.setInterval(200);
        timer.setSingleShot(true);

        QList<QMetaObject::Connection> conns;
        conns.push_back(QObject::connect(this,&Client::answer_received_internal,[&ret,&el](QByteArray data) { ret = data; el.quit(); }));
        conns.push_back(QObject::connect(&timer,&QTimer::timeout,[&msg]() { std::cerr << "Request timed out: " << msg.toString().toStdString() << std::endl; }));
        conns.push_back(QObject::connect(&timer,&QTimer::timeout,&el,&QEventLoop::quit));

        timer.start();

        emit send_msg(this,msg);

        el.exec();

        foreach (QMetaObject::Connection con, conns) {
            QObject::disconnect(con);
        }
    }
    else {
        emit send_msg(this,msg);
    }

    return ret;
}
