#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "peripherals/roboclaw/roboclaw.h"
#include "ui/menu_user.h"
#include "utils/settings.h"
#include <QString>

class Actuator : public QObject, public RC::RoboClaw, public MenuUser {
    Q_OBJECT
public:
    Actuator(QString name);
    virtual ~Actuator() {}

    void connect(QString default_port_name, unsigned int default_baudrate, int default_address, RC::RoboClaw::Channel default_channel);

    virtual void calibrate() {}

    double pos();
    using RC::RoboClaw::move_to;
    void move_to(double deg, double speed, bool block = false);
    void set_velocity(double deg_s);
    virtual void set_velocity_safe(double deg_s);

protected:
    void calibrate(double velocity_deg_s, double final_pos, double velocity_threshold_deg_s, bool use_velocity_control = true);
    void read_params_limits(double default_lower_limit_deg, double default_upper_limit_deg);
    void read_params_technical(unsigned int default_incs_per_deg, int default_deg_s2);
    void read_params_velocity(double default_kp, double default_ki, double default_kd, unsigned int default_qpps);
    void read_params_position(double default_kp, double default_ki, double default_kd, double default_i_max, int default_deadzone, int default_lower_limit, int default_upper_limit);

    Settings _settings;

protected slots:
    virtual void on_exit();

private:
    QString _name;

    bool _connected;

    unsigned int _incs_per_deg;

    bool _calibrated;

    int _acc;

    double _min_angle;
    double _max_angle;
};

#endif // ACTUATOR_H
