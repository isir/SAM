#include "actuator.h"
#include <QCoreApplication>
#include <QDebug>
#include <QTime>

Actuator::Actuator(QString name, std::shared_ptr<QMqttClient> mqtt)
    : RoboClaw::RoboClaw()
    , _menu(mqtt)
    , _name(name)
    , _connected(false)
    , _incs_per_deg(0)
    , _calibrated(false)
{
    _settings.beginGroup(_name);

    QObject::connect(&_menu, &ConsoleMenu::finished, this, &Actuator::on_exit);

    _menu.addItem(ConsoleMenuItem("Forward (0-127)", "f", [this](QString args) { if(args.isEmpty()) args = "20"; forward(args.toUInt()); }));
    _menu.addItem(ConsoleMenuItem("Backward (0-127)", "b", [this](QString args) { if(args.isEmpty()) args = "20"; backward(args.toUInt()); }));
    _menu.addItem(ConsoleMenuItem("Print current", "pc", [this](QString) { qInfo() << "Current:" << read_current() << "A"; }));
    _menu.addItem(ConsoleMenuItem("Print firmware version", "fw", [this](QString) { qInfo() << read_firmware_version(); }));
    _menu.addItem(ConsoleMenuItem("Print encoder speed", "es", [this](QString) { qInfo() << "Speed:" << read_encoder_speed() << "steps/s"; }));
    _menu.addItem(ConsoleMenuItem("Stop", "s", [this](QString) { forward(0); }));
    _menu.addItem(ConsoleMenuItem("Calibrate", "calib", [this](QString) { calibrate(); }));
    _menu.addItem(ConsoleMenuItem("Read encoder", "e", [this](QString) { qInfo() << "Position:" << read_encoder_position() << "steps"; }));
    _menu.addItem(ConsoleMenuItem("Go to", "g", [this](QString args) { if(!args.isEmpty()) move_to(args.toDouble(), 10); }));
    _menu.addItem(ConsoleMenuItem("Set velocity (deg/s)", "v", [this](QString args) { if(args.isEmpty()) args = "0"; set_velocity_safe(args.toInt()); }));
    _menu.addItem(ConsoleMenuItem("Set encoder zero", "z", [this](QString) { set_encoder_position(0); }));
}

void Actuator::connect(QString default_port_name, unsigned int default_baudrate, int default_address, RoboClaw::RoboClaw::Channel default_channel)
{
    int address = _settings.value("address", default_address).toInt();
    Channel channel = static_cast<Channel>(_settings.value("channel", default_channel).toInt());
    QString port_name = _settings.value("port_name", default_port_name).toString();
    unsigned int baudrate = _settings.value("baudrate", default_baudrate).toUInt();

    init(port_name, baudrate, address, channel);

    _connected = true;
}

double Actuator::pos()
{
    return static_cast<double>(read_encoder_position()) / _incs_per_deg;
}

void Actuator::move_to(double deg, double speed, bool block)
{
    if (!_calibrated) {
        qWarning() << "Not calibrated...";
        return;
    }

    qint32 target = static_cast<qint32>(deg * _incs_per_deg);
    quint32 velocity = static_cast<quint32>(speed * _incs_per_deg);

    if (velocity == 0)
        velocity = 1;

    RoboClaw::RoboClaw::move_to(_acc, velocity, _acc, target);

    if (block) {
        int threshold = 10000;
        do {
            QThread::msleep(10);
            QCoreApplication::processEvents();
        } while ((qAbs(target - read_encoder_position()) > threshold));
    }
}

void Actuator::set_velocity(double deg_s)
{
    RoboClaw::RoboClaw::set_velocity(qRound(deg_s * _incs_per_deg));
}

void Actuator::set_velocity_safe(double deg_s)
{
    if (!_calibrated) {
        qWarning() << "Not calibrated...";
        return;
    }

    move_to(deg_s > 0 ? _max_angle : _min_angle, qAbs(deg_s));
}

void Actuator::calibrate(double velocity_deg_s, double final_pos, double velocity_threshold_deg_s, bool use_velocity_control)
{
    int calib_velocity_threshold = velocity_threshold_deg_s * _incs_per_deg;

    QTime t;

    if (use_velocity_control) {
        set_velocity(velocity_deg_s);
    } else {
        if (velocity_deg_s < 0) {
            backward(qAbs(velocity_deg_s));
        } else {
            forward(velocity_deg_s);
        }
    }

    t.start();
    while ((qAbs(read_encoder_speed()) < calib_velocity_threshold) && t.elapsed() < 500)
        QCoreApplication::processEvents();
    while (qAbs(read_encoder_speed()) > calib_velocity_threshold)
        QCoreApplication::processEvents();

    set_encoder_position(qRound(final_pos * _incs_per_deg));
    forward(0);
    _calibrated = true;

    RC::position_pid_params_t p_params = read_position_pid();
    _settings.beginGroup("Position_PID");
    p_params.min_pos = _settings.value("min_pos", _min_angle * _incs_per_deg).toInt();
    p_params.max_pos = _settings.value("max_pos", _max_angle * _incs_per_deg).toInt();
    _settings.endGroup();

    set_position_pid(p_params);
}

void Actuator::read_params_limits(double default_lower_limit_deg, double default_upper_limit_deg)
{
    _min_angle = _settings.value("min_angle", default_lower_limit_deg).toDouble();
    _max_angle = _settings.value("max_angle", default_upper_limit_deg).toDouble();
}

void Actuator::read_params_technical(unsigned int default_incs_per_deg, int default_deg_s2)
{
    _incs_per_deg = _settings.value("incs_per_deg", default_incs_per_deg).toInt();
    _acc = _settings.value("accel_decel", default_deg_s2 * _incs_per_deg).toDouble();
}

void Actuator::read_params_velocity(double default_kp, double default_ki, double default_kd, unsigned int default_qpps)
{
    if (!_connected) {
        qWarning() << "Need to connect to a RoboClaw server first !";
        return;
    }

    RC::velocity_pid_params_t v_params = {};
    _settings.beginGroup("Velocity_PID");
    v_params.p = _settings.value("kp", default_kp).toDouble();
    v_params.i = _settings.value("ki", default_ki).toDouble();
    v_params.d = _settings.value("kd", default_kd).toDouble();
    v_params.qpps = _settings.value("qpps", default_qpps).toUInt();
    _settings.endGroup();
    set_velocity_pid(v_params);
}

void Actuator::read_params_position(double default_kp, double default_ki, double default_kd, double default_i_max, int default_deadzone, int default_lower_limit, int default_upper_limit)
{
    if (!_connected) {
        qWarning() << "Need to connect to a RoboClaw server first !";
        return;
    }

    RC::position_pid_params_t p_params = {};
    _settings.beginGroup("Position_PID");
    p_params.p = _settings.value("kp", default_kp).toDouble();
    p_params.i = _settings.value("ki", default_ki).toDouble();
    p_params.d = _settings.value("kd", default_kd).toDouble();
    p_params.i_max = _settings.value("i_max", default_i_max).toDouble();
    p_params.deadzone = _settings.value("deadzone", default_deadzone).toInt();
    p_params.min_pos = default_lower_limit * _incs_per_deg;
    p_params.max_pos = default_upper_limit * _incs_per_deg;
    set_position_pid(p_params);
}

void Actuator::on_exit()
{
    forward(0);
}
