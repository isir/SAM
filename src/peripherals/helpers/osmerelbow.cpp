#include "osmerelbow.h"
#include "peripherals/roboclaw/factory.h"
#include "peripherals/roboclaw/server.h"
#include <iostream>
#include <QCoreApplication>
#include <QTime>
#include <QThread>
#include <QDebug>

OsmerElbow::OsmerElbow() : RoboClaw::Client(),
    _calibrated(false)
{
    _settings.beginGroup("Osmer");
    _calibration_velocity_threshold = _settings.value("calibration_velocity_threshold",1000).toInt();
    _incs_per_deg = _settings.value("incs_per_deg",23422).toInt();
    _min_angle = _settings.value("min_angle",-100.).toDouble();
    _max_angle = _settings.value("max_angle",0.).toDouble();
    int address = _settings.value("address", 0x80).toInt();
    Channel channel = static_cast<Channel>(_settings.value("channel",1).toInt());
    QString port_name = _settings.value("port_name","/dev/ttyAMA0").toString();
    int baudrate = _settings.value("baudrate",230400).toInt();

    set_address(address,channel);
    connect_to_server(port_name, baudrate);

    QObject::connect(&_menu,&ConsoleMenu::finished,this,&OsmerElbow::on_exit);

    _menu.set_title(QString("Osmer - ") + read_firmware_version() + " - " + port_name + " - " + QString::number(baudrate) + " - " + QString::number(address) + "/" + QString::number(chan()));
    _menu.set_code(QString("osmer"));

    _menu.addItem(ConsoleMenuItem("Forward","f",[this](QString){ this->forward(10); }));
    _menu.addItem(ConsoleMenuItem("Backward","b",[this](QString){ this->backward(10); }));
    _menu.addItem(ConsoleMenuItem("Print current","pc",[this](QString){ std::cout << "Current: " << this->read_current() << "A" << std::endl; }));
    _menu.addItem(ConsoleMenuItem("Print encoder speed","es",[this](QString){ std::cout << "Speed: " << this->read_encoder_speed() << std::endl; }));
    _menu.addItem(ConsoleMenuItem("Stop","s",[this](QString){ this->forward(0); }));
    _menu.addItem(ConsoleMenuItem("Calibrate","calib",[this](QString){ this->calibration(); }));
    _menu.addItem(ConsoleMenuItem("Read encoder","e",[this](QString){ std::cout << this->read_encoder_position() << std::endl; }));
    _menu.addItem(ConsoleMenuItem("Go to","g",[this](QString args){ if(!args.isEmpty()) this->move_to_angle(args.toInt(),10); }));
    _menu.addItem(ConsoleMenuItem("Set velocity (deg/s)","v",[this](QString args = "0"){ if(!args.isEmpty()) this->set_velocity(args.toInt()); }));

    RoboClaw::velocity_pid_params_t v_params = {};
    _settings.beginGroup("Velocity_PID");
    v_params.p = _settings.value("kp",0.0387).toDouble();
    v_params.i = _settings.value("ki",0.0029).toDouble();
    v_params.d = _settings.value("kd",0.).toDouble();
    v_params.qpps = _settings.value("qpps",1295000).toUInt();
    _settings.endGroup();
    set_velocity_pid(v_params);

    RoboClaw::position_pid_params_t p_params = {};
    _settings.beginGroup("Position_PID");
    p_params.p = _settings.value("kp",68.).toDouble();
    p_params.i = _settings.value("ki",0.).toDouble(); //1.6
    p_params.d = _settings.value("kd",0.).toDouble(); // 473
    p_params.i_max = _settings.value("i_max",13000.).toDouble();
    p_params.deadzone = _settings.value("deadzone",0).toInt();
    p_params.min_pos = -180*_incs_per_deg;
    p_params.max_pos = 180*_incs_per_deg;
    set_position_pid(p_params);

    _settings.sync();
}

OsmerElbow::~OsmerElbow() {

}

OsmerElbow& OsmerElbow::instance() {
    static OsmerElbow _instance;
    return _instance;
}

ConsoleMenu& OsmerElbow::menu() {
    return _menu;
}

/**
 * \brief OsmerElbow::calibration Performs the Osmer calibration. Moves slowly to the down stop and resets enc
 */
void OsmerElbow::calibration() {
    if(_calibrated)
        return;

    QTime t;

    forward(10);
    t.start();
    while((qAbs(read_encoder_speed()) < _calibration_velocity_threshold) && t.elapsed() < 500) QCoreApplication::processEvents();
    while(qAbs(read_encoder_speed()) > _calibration_velocity_threshold)  QCoreApplication::processEvents();

    set_encoder_position(4 * _incs_per_deg);
    forward(0);
    _calibrated = true;

    move_to_angle(-90,20);

    RoboClaw::position_pid_params_t p_params = read_position_pid();
    _settings.beginGroup("Position_PID");
    p_params.min_pos = _settings.value("min_pos",_min_angle*_incs_per_deg).toInt();
    p_params.max_pos = _settings.value("max_pos",_max_angle*_incs_per_deg).toInt();
    _settings.endGroup();

    set_position_pid(p_params);
}

double OsmerElbow::angle() {
    return static_cast<double>(read_encoder_position())/_incs_per_deg;
}

/**
 * \brief OsmerElbow::move_to_angle
 * \param angle (degs)
 * \param speed Default is 30°/s
 * \return
 */
void OsmerElbow::move_to_angle(double angle, double speed, bool block) {
    if(!_calibrated)
        return;

    qint32 target = static_cast<qint32>(angle*_incs_per_deg);
    move_to(qRound(speed*_incs_per_deg*2),qRound(speed*_incs_per_deg),qRound(speed*_incs_per_deg*2),target);

    if(block) {
        int threshold = 10000;
        do {
            QThread::msleep(10);
            QCoreApplication::processEvents();
        } while((qAbs(target - read_encoder_position()) > threshold));
    }
}

/**
 * \brief OsmerElbow::set_velocity Controlled speed funtion with software stops.
 * \param value velocity (degs/s)
 */
void OsmerElbow::set_velocity(double value) {
    static bool locked = false;

    if(!_calibrated)
        return;

    if (value == 0 && !locked){
        move_to_angle(angle());
        locked = true;
    }
    else if(value != 0) {
        move_to_angle(value > 0 ? _max_angle : _min_angle, qAbs(value));
        locked = false;
    }
}

void OsmerElbow::on_exit() {
    forward(0);
}

