#include "wristflexor.h"
#include "peripherals/roboclaw/factory.h"
#include "peripherals/roboclaw/server.h"
#include <QCoreApplication>
#include <QDebug>
#include <QThread>
#include <QTime>
#include <iostream>

WristFlexor::WristFlexor(std::shared_ptr<QMqttClient> mqtt)
    : RoboClaw::Client()
    , _menu(mqtt)
    , _calibrated(false)
{
    _settings.beginGroup("WristFlexor");
    _calibration_velocity_threshold = _settings.value("calibration_velocity_threshold", 1000).toInt();
    _incs_per_deg = _settings.value("incs_per_deg", 12000).toInt();
    _min_angle = _settings.value("min_angle", -35.).toDouble();
    _max_angle = _settings.value("max_angle", 35.).toDouble();
    _accel_decel = _settings.value("accel_decel", 100 * _incs_per_deg).toDouble(); //100degres/sec² (in one second)
    int address = _settings.value("address", 0x81).toInt();
    Channel channel = static_cast<Channel>(_settings.value("channel", 2).toInt());
    QString port_name = _settings.value("port_name", "/dev/ttyAMA0").toString();
    int baudrate = _settings.value("baudrate", 230400).toInt();

    set_address(address, channel);
    connect_to_server(port_name, baudrate);

    QObject::connect(&_menu, &ConsoleMenu::finished, this, &WristFlexor::on_exit);

    _menu.set_title(QString("Wrist Flexor - ") + read_firmware_version() + " - " + port_name + " - " + QString::number(baudrate) + " - " + QString::number(address) + "/" + QString::number(chan()));
    _menu.set_code(QString("flex"));

    _menu.addItem(ConsoleMenuItem("Forward", "f", [this](QString) { this->forward(10); }));
    _menu.addItem(ConsoleMenuItem("Backward", "b", [this](QString) { this->backward(10); }));
    _menu.addItem(ConsoleMenuItem("Print current", "pc", [this](QString) { std::cout << "Current: " << this->read_current() << "A" << std::endl; }));
    _menu.addItem(ConsoleMenuItem("Print firmware version", "fw", [this](QString) { std::cout << this->read_firmware_version().toStdString() << std::endl; }));
    _menu.addItem(ConsoleMenuItem("Print encoder speed", "es", [this](QString) { std::cout << "Speed: " << this->read_encoder_speed() << std::endl; }));
    _menu.addItem(ConsoleMenuItem("Stop", "s", [this](QString) { this->forward(0); }));
    _menu.addItem(ConsoleMenuItem("Calibrate", "calib", [this](QString) { this->calibration(); }));
    _menu.addItem(ConsoleMenuItem("Read encoder", "e", [this](QString) { std::cout << this->read_encoder_position() << std::endl; }));
    _menu.addItem(ConsoleMenuItem("Go to", "g", [this](QString args) { if(!args.isEmpty()) this->move_to_angle(args.toInt(),10); }));
    _menu.addItem(ConsoleMenuItem("Set velocity (deg/s)", "v", [this](QString args = "0") { if(!args.isEmpty()) this->set_velocity(args.toInt()); }));

    RoboClaw::velocity_pid_params_t v_params = {};
    _settings.beginGroup("Velocity_PID");
    v_params.p = _settings.value("kp", 0.177).toDouble();
    v_params.i = _settings.value("ki", 0.012).toDouble();
    v_params.d = _settings.value("kd", 0.).toDouble();
    v_params.qpps = _settings.value("qpps", 335500).toUInt();
    _settings.endGroup();
    set_velocity_pid(v_params);

    RoboClaw::position_pid_params_t p_params = {};
    _settings.beginGroup("Position_PID");
    p_params.p = _settings.value("kp", 20.).toDouble();
    p_params.i = _settings.value("ki", 0.).toDouble(); //1.6
    p_params.d = _settings.value("kd", 0.).toDouble(); // 473
    p_params.i_max = _settings.value("i_max", 0.).toDouble();
    p_params.deadzone = _settings.value("deadzone", 0).toInt();
    p_params.min_pos = -180 * _incs_per_deg;
    p_params.max_pos = 180 * _incs_per_deg;
    set_position_pid(p_params);

    _settings.sync();
}

ConsoleMenu& WristFlexor::menu()
{
    return _menu;
}

void WristFlexor::calibration()
{
    if (_calibrated)
        return;

    QTime t;

    backward(30);
    t.start();
    while ((qAbs(read_encoder_speed()) < _calibration_velocity_threshold) && t.elapsed() < 500) {
        QCoreApplication::processEvents();
    }
    while (qAbs(read_encoder_speed()) > _calibration_velocity_threshold) {
        QCoreApplication::processEvents();
    }

    set_encoder_position(-43 * _incs_per_deg);
    forward(0);
    _calibrated = true;

    move_to_angle(0, 20);

    RoboClaw::position_pid_params_t p_params = read_position_pid();
    _settings.beginGroup("Position_PID");
    p_params.min_pos = _settings.value("min_pos", _min_angle * _incs_per_deg).toInt();
    p_params.max_pos = _settings.value("max_pos", _max_angle * _incs_per_deg).toInt();
    _settings.endGroup();

    set_position_pid(p_params);
}

double WristFlexor::angle()
{
    return static_cast<double>(read_encoder_position()) / _incs_per_deg;
}

void WristFlexor::move_to_angle(double angle, double speed, bool block)
{
    if (!_calibrated)
        return;

    qint32 target = static_cast<qint32>(angle * _incs_per_deg);
    quint32 velocity = static_cast<quint32>(speed * _incs_per_deg);

    if (velocity == 0)
        velocity = 1;

    move_to(_accel_decel, velocity, _accel_decel, target);

    if (block) {
        int threshold = 10000;
        do {
            QThread::msleep(10);
            QCoreApplication::processEvents();
        } while ((qAbs(target - read_encoder_position()) > threshold));
    }
}

void WristFlexor::set_velocity(double value)
{
    if (!_calibrated)
        return;

    move_to_angle(value > 0 ? _max_angle : _min_angle, qAbs(value));
}

void WristFlexor::on_exit()
{
    forward(0);
}
